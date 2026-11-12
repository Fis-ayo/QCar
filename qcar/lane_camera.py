"""
lane_camera.py

Camera lane correction. Reads the front CSI camera, finds the yellow divider
(left) and the right road edge, projects them onto the floor and logs the
car's sideways offset from the lane centre. It never touches the motors.
Step 5 (2026-11-11): each result is also sent over UDP to 127.0.0.1:--udp-port
(text "t_epoch,mode,offset_cm,heading_deg,conf"). vehicle_control.py uses it
when its use_camera flag is on; start this script BEFORE vehicle_control.py.

Run on the QCar, in a second SSH window, alongside a normal drive:
    python -u lane_camera.py
    python -u lane_camera.py --duration 120 --debug-every 1.0
Stop with Ctrl+C. Output:
    lane_camera.csv     one row per frame (columns: see LOG_COLUMNS)
    lane_debug/*.jpg    an overlay image every --debug-every seconds

Sign conventions (car frame, origin at the centre of the front axle):
    offset_cm   + = car is LEFT of the lane centre (as in check_pose.py)
    heading_deg + = car points LEFT of the lane direction
Calibration: docs/plans/camera_lane_correction.md, step 1.
"""
import argparse
import os
import socket
import time
import numpy as np
import cv2

# -- Calibration (front CSI camera, car A, 2026-11-10) --------------------------
IMG_W, IMG_H = 820, 410
K = np.array([[495.84, 0.00, 408.03],
              [0.00, 454.60, 181.21],
              [0.00, 0.00, 1.00]])
DIST = np.array([-0.57513, 0.37175, -0.00489, -0.00277, -0.11136])
PITCH = np.radians(-2.67)   # from the vanishing point of the straight snapshots
CAM_H = 0.11                # m, camera above the floor (ruler)
CAM_AHEAD = 0.10            # m, camera ahead of the front axle (ruler)
LANE_W = 0.257              # m, yellow line centre to road edge (ruler)
RAW_LANE_W = 0.355          # m, lane width the uncorrected projection gives
SCALE = LANE_W / RAW_LANE_W # fixed scale when only one line is visible

# -- Detection settings ---------------------------------------------------------
YELLOW_LO = np.array([10, 50, 100])   # from examples/QCar2_task_lane_following_probe.py
YELLOW_HI = np.array([45, 255, 255])
FX, FY, CX, CY = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
HORIZON_V = CY - FY * np.tan(PITCH)   # image row of the horizon
ROI_TOP = int(HORIZON_V) + 18         # ignore everything above (room, boxes)
FIT_MAX_FWD = 0.70                    # m (raw scale), fit only the near part
MIN_POINTS = 40                       # pixels needed to accept a line
MAX_RMS = 0.015                       # m, line fit scatter limit
WIDTH_TOL = 0.20                      # accepted lane width error (fraction)
HEADING_MAX = 10.0                    # deg; beyond this the fit is a curve chord, not the lane at the car
CURV_MAX = 0.5                        # 1/m; tighter curves can't be extrapolated back to the axle
EDGE_BRIGHT, EDGE_DARK = 130, 110     # V levels: white surface / dark road

LOG_COLUMNS = ['t_epoch', 't', 'mode', 'offset_cm', 'heading_deg', 'conf',
               'yellow_cm', 'edge_cm', 'width_cm', 'curv_1pm',
               'n_yellow', 'n_edge', 'rms_yellow_cm', 'rms_edge_cm', 'proc_ms']

MAP1, MAP2 = cv2.initUndistortRectifyMap(K, DIST, None, K, (IMG_W, IMG_H), cv2.CV_16SC2)


def to_ground(v, u):
    """Undistorted pixel rows/cols -> (forward, left) in m, car frame, raw scale."""
    x = (u - CX) / FX
    y = (v - CY) / FY
    t = CAM_H / (np.sin(PITCH) + y * np.cos(PITCH))
    fwd = t * (np.cos(PITCH) - y * np.sin(PITCH)) + CAM_AHEAD
    return fwd, -t * x


def robust_line(f, l, min_pts=MIN_POINTS, iters=4):
    """Fit l = a*f + b, dropping outliers. Returns (a, b, rms, n_inliers, keep) or None."""
    keep = np.ones(len(f), bool)
    for _ in range(iters):
        if keep.sum() < min_pts:
            return None
        a, b = np.polyfit(f[keep], l[keep], 1)
        r = np.abs(l - (a * f + b))
        keep = r < max(0.01, 2.5 * np.median(r[keep]))
    if keep.sum() < min_pts:
        return None
    a, b = np.polyfit(f[keep], l[keep], 1)
    rms = float(np.sqrt(np.mean((l[keep] - (a * f[keep] + b)) ** 2)))
    return a, b, rms, int(keep.sum()), keep


def find_yellow(hsv):
    """Yellow pixels (colour AND near an edge) below the horizon -> ground points."""
    roi = hsv[ROI_TOP:]
    mask = cv2.inRange(roi, YELLOW_LO, YELLOW_HI)
    v = roi[:, :, 2]
    gx = cv2.Sobel(v, cv2.CV_16S, 1, 0, ksize=3)
    gy = cv2.Sobel(v, cv2.CV_16S, 0, 1, ksize=3)
    edges = ((np.abs(gx) + np.abs(gy)) > 60).astype(np.uint8)
    edges = cv2.dilate(edges, np.ones((7, 7), np.uint8))   # cover the line's interior
    mask &= edges * 255
    vs, us = np.nonzero(mask[::2])                          # every 2nd row is enough
    vs = vs * 2 + ROI_TOP
    return vs.astype(float), us.astype(float), mask


def find_edge(hsv, yellow_img_fit):
    """Right road edge: per row, first dark->bright step right of the yellow line."""
    V = cv2.blur(hsv[:, :, 2], (3, 3)).astype(int)
    ev, eu = [], []
    for r in range(ROI_TOP + 10, IMG_H - 4, 2):
        u0 = int(yellow_img_fit(r)) + 15 if yellow_img_fit else int(CX) - 60
        u0 = max(u0, 0)
        if u0 >= IMG_W - 5:
            continue
        row = V[r, u0:]
        bright = (row[:-3] > EDGE_BRIGHT) & (row[1:-2] > EDGE_BRIGHT) & (row[3:] > EDGE_BRIGHT)
        idx = np.nonzero(bright)[0]
        if len(idx) == 0:
            continue
        i = idx[0]
        if i < 3 or row[max(i - 6, 0)] > EDGE_DARK:   # must step up from dark road
            continue
        ev.append(r)
        eu.append(u0 + i)
    return np.array(ev, float), np.array(eu, float)


def process(bgr):
    """One frame -> dict of measurements (log row without times) + debug data."""
    t0 = time.time()
    und = cv2.remap(bgr, MAP1, MAP2, cv2.INTER_LINEAR)
    hsv = cv2.cvtColor(und, cv2.COLOR_BGR2HSV)
    out = dict(mode='none', offset_cm=np.nan, heading_deg=np.nan, conf=0.0,
               yellow_cm=np.nan, edge_cm=np.nan, width_cm=np.nan, curv_1pm=np.nan,
               n_yellow=0, n_edge=0, rms_yellow_cm=np.nan, rms_edge_cm=np.nan)
    dbg = dict(und=und, yellow=None, edge=None)

    # Yellow line
    yv, yu, ymask = find_yellow(hsv)
    dbg['ymask'] = ymask
    yfit, img_fit = None, None
    if len(yv) >= MIN_POINTS:
        f, l = to_ground(yv, yu)
        near = (f > 0) & (f < FIT_MAX_FWD + CAM_AHEAD) & (l > 0)   # divider is on the left
        if near.sum() >= MIN_POINTS:
            yfit = robust_line(f[near], l[near])
            if yfit and yfit[2] > MAX_RMS:
                yfit = None
        if yfit:
            a_img, b_img = np.polyfit(yv[near][yfit[4]], yu[near][yfit[4]], 1)
            img_fit = lambda r: a_img * r + b_img
            dbg['yellow'] = (yv[near][yfit[4]], yu[near][yfit[4]])
            # curvature from a quadratic over all ground points (for logging)
            ff, ll = f[near][yfit[4]], l[near][yfit[4]]
            if ff.max() - ff.min() > 0.25:
                out['curv_1pm'] = 2 * np.polyfit(ff, ll, 2)[0] / SCALE

    # Right road edge
    ev, eu = find_edge(hsv, img_fit)
    efit = None
    if len(ev) >= MIN_POINTS // 2:
        f, l = to_ground(ev, eu)
        near = (f > 0) & (f < FIT_MAX_FWD + CAM_AHEAD) & (l < 0)    # edge is on the right
        if near.sum() >= MIN_POINTS // 2:
            efit = robust_line(f[near], l[near], min_pts=MIN_POINTS // 2)
            if efit and efit[2] > MAX_RMS:
                efit = None
        if efit:
            dbg['edge'] = (ev[near][efit[4]], eu[near][efit[4]])

    # Combine. Raw-scale lateral distances at the front axle: yl (left, >0), el (right, <0)
    if yfit:
        out.update(yellow_cm=100 * yfit[1] * SCALE, n_yellow=yfit[3], rms_yellow_cm=100 * yfit[2] * SCALE)
    if efit:
        out.update(edge_cm=100 * efit[1] * SCALE, n_edge=efit[3], rms_edge_cm=100 * efit[2] * SCALE)
    conf_y = min(1.0, yfit[3] / 300) if yfit else 0.0
    conf_e = min(1.0, efit[3] / 40) if efit else 0.0
    width_ok = False
    if yfit and efit:
        raw_w = yfit[1] - efit[1]
        out['width_cm'] = 100 * raw_w * SCALE
        width_ok = abs(raw_w - RAW_LANE_W) < WIDTH_TOL * RAW_LANE_W
    if width_ok:
        # Both lines: the known lane width fixes the scale on every frame
        centre = (yfit[1] + efit[1]) / 2 * LANE_W / raw_w
        out.update(mode='both', offset_cm=-100 * centre,
                   heading_deg=-np.degrees(np.arctan((yfit[0] + efit[0]) / 2)),
                   conf=0.5 + 0.5 * min(conf_y, conf_e))
    elif yfit:
        out.update(mode='yellow', offset_cm=100 * (LANE_W / 2 - yfit[1] * SCALE),
                   heading_deg=-np.degrees(np.arctan(yfit[0])), conf=0.5 * conf_y)
    elif efit:
        out.update(mode='edge', offset_cm=-100 * (efit[1] * SCALE + LANE_W / 2),
                   heading_deg=-np.degrees(np.arctan(efit[0])), conf=0.3 * conf_e)
    # Curves: the camera only sees the line from ~30 cm ahead, so on a tight
    # curve the offset at the axle can't be trusted. Log the raw values, but
    # flag the frame and give it no confidence.
    if out['mode'] != 'none' and (abs(out['heading_deg']) > HEADING_MAX
                                  or abs(np.nan_to_num(out['curv_1pm'])) > CURV_MAX):
        out.update(mode='curve', offset_cm=np.nan, heading_deg=np.nan, conf=0.0)
    out['proc_ms'] = 1000 * (time.time() - t0)
    return out, dbg


def draw_debug(out, dbg):
    img = dbg['und'].copy()
    cv2.line(img, (0, ROI_TOP), (IMG_W, ROI_TOP), (255, 255, 0), 1)
    if dbg['yellow'] is not None:
        for v, u in zip(*dbg['yellow']):
            img[int(v), int(u)] = (0, 0, 255)
    if dbg['edge'] is not None:
        for v, u in zip(*dbg['edge']):
            cv2.circle(img, (int(u), int(v)), 2, (255, 0, 0), -1)
    txt = f"{out['mode']}  off={out['offset_cm']:+.1f}cm  head={out['heading_deg']:+.1f}deg  conf={out['conf']:.2f}  w={out['width_cm']:.1f}cm"
    cv2.putText(img, txt, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    return img


def fmt(x):
    if isinstance(x, str):
        return x
    if isinstance(x, (int, np.integer)):
        return str(int(x))
    return 'nan' if np.isnan(x) else f'{x:.3f}'


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--duration', type=float, default=0, help='seconds, 0 = until Ctrl+C')
    ap.add_argument('--rate', type=int, default=30, help='camera frame rate')
    ap.add_argument('--max-rate', type=float, default=15, help='max frames processed per second')
    ap.add_argument('--debug-every', type=float, default=2.0, help='seconds between overlay images, 0 = off')
    ap.add_argument('--log', default='lane_camera.csv')
    ap.add_argument('--udp-port', type=int, default=18555, help='send results to vehicle_control.py on 127.0.0.1, 0 = off')
    args = ap.parse_args()
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) if args.udp_port else None

    from pal.products.qcar import QCarCameras
    cams = QCarCameras(frameWidth=IMG_W, frameHeight=IMG_H, frameRate=args.rate, enableFront=True)
    os.makedirs('lane_debug', exist_ok=True)
    for f in os.listdir('lane_debug'):          # don't mix images from earlier runs
        if f.endswith('.jpg'):
            os.remove(os.path.join('lane_debug', f))
    logf = open(args.log, 'w')
    logf.write(','.join(LOG_COLUMNS) + '\n')
    t_start = time.time()
    last_dbg = last_print = last_proc = 0.0
    n = 0
    try:
        while not args.duration or time.time() - t_start < args.duration:
            # Process only new frames, at most --max-rate per second, so the
            # GPS/lidar localization isn't starved of CPU (2026-11-10 run:
            # 62 % of processed frames were repeats at ~70 Hz).
            if time.time() - last_proc < 1.0 / args.max_rate:
                time.sleep(0.005)
                continue
            flags = cams.readAll()
            img = cams.csiFront.imageData
            if not any(flags) or img is None or not img.any():
                time.sleep(0.005)
                continue
            now = time.time()
            last_proc = now
            out, dbg = process(img)
            out['t_epoch'], out['t'] = now, now - t_start
            logf.write(','.join(fmt(out[c]) for c in LOG_COLUMNS) + '\n')
            if sock:
                msg = ','.join(fmt(out[c]) for c in ['t_epoch', 'mode', 'offset_cm', 'heading_deg', 'conf'])
                sock.sendto(msg.encode(), ('127.0.0.1', args.udp_port))
            n += 1
            if args.debug_every and now - last_dbg >= args.debug_every:
                cv2.imwrite(f"lane_debug/{out['t']:07.2f}.jpg", draw_debug(out, dbg))
                last_dbg = now
            if now - last_print >= 1.0:
                print(f"t={out['t']:6.1f}  {out['mode']:6s}  offset={out['offset_cm']:+6.1f} cm  "
                      f"heading={out['heading_deg']:+5.1f} deg  conf={out['conf']:.2f}  "
                      f"width={out['width_cm']:5.1f} cm  {out['proc_ms']:.0f} ms/frame")
                last_print = now

    except KeyboardInterrupt:
        pass
    finally:
        logf.close()
        cams.terminate()
        print(f'{n} frames logged to {args.log}')


if __name__ == '__main__':
    main()
