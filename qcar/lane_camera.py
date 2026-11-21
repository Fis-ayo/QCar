"""
lane_camera.py

Camera lane correction. Reads the front CSI camera, finds the yellow divider
(left) and the right road edge, projects them onto the floor and logs the
car's sideways offset from the lane centre. It never touches the motors.
Step 5 (2026-11-11): each result is also sent over UDP to 127.0.0.1:--udp-port
(text "t_epoch,mode,offset_cm,heading_deg,conf,mf_src,mf_d_cm,mf_h_deg,mf_rms_cm,mf_s").
Step C3 (2026-11-18, camera report §35): when vehicle_control.py (use_camera_steer)
sends the map's lane curvature to PROFILE_PORT, each frame is also fitted with
that lane shape (the mf_* fields); vehicle_control.py steers from those.
Start this script BEFORE vehicle_control.py.

Run on the QCar, in a second SSH window, alongside a normal drive:
    python -u lane_camera.py
    python -u lane_camera.py --duration 120 --debug-every 1.0
    python -u lane_camera.py --save-raw --udp-port 0     (raw frames for offline curve tests)
Stop with Ctrl+C. Output:
    lane_camera.csv     one row per frame (columns: see LOG_COLUMNS)
    lane_debug/*.jpg    an overlay image every --debug-every seconds
    raw_frames/<start time>/<t_epoch>_front.jpg, _left.jpg   with --save-raw,
                        every --raw-every s (2026-11-16, camera report §15)

Sign conventions (car frame, origin at the centre of the front axle):
    offset_cm   + = car is LEFT of the lane centre (as in check_pose.py)
    heading_deg + = car points LEFT of the lane direction
Calibration: checkerboard + strip image, docs/history/2026-11-10_camera_lane_correction.md §28-§29.
"""
import argparse
import os
import socket
import time
import numpy as np
import cv2

# -- Calibration (front CSI camera, car A, 2026-11-18, camera report §28-§29) ---
# Intrinsics: checkerboard (qcar/calib_solve.py, 147 frames, rms 0.19 px). The
# pixels aren't square (FY/FX = 0.665: the 4:3 sensor is squeezed into 820x410).
# Mounting: from the paper-strip image (qcar/road_check.py): pitch from the lane
# lines' vanishing point, height from the lane width, CAM_AHEAD from the strips
# (30/45/60 cm -> 29.8/46.0/59.2). True scale: lane width 25.7 cm and the curve
# radius (edge 0.95-1.07 m, map 1.02 m) come out right without a scale factor.
# Replaces the borrowed intrinsics + x0.724 scale used until 2026-11-18.
IMG_W, IMG_H = 820, 410
K = np.array([[388.89, 0.00, 411.34],
              [0.00, 258.50, 201.73],
              [0.00, 0.00, 1.00]])
DIST = np.array([-0.38827, 0.24861, -0.00028, 0.00010, -0.10079])
PITCH = np.radians(-0.56)   # from the vanishing point of the lane lines (strip image)
CAM_H = 0.105               # m, lens above the floor (from the lane width; ruler 11 cm to the top)
CAM_AHEAD = 0.062           # m, camera ahead of the front axle (from the strips)
LANE_W = 0.257              # m, yellow line centre to road edge (ruler)
RAW_LANE_W = LANE_W         # m, lane width the projection gives (true scale now)
SCALE = LANE_W / RAW_LANE_W # = 1; kept so older code and logs stay comparable

# -- Detection settings ---------------------------------------------------------
YELLOW_LO = np.array([10, 50, 100])   # from examples/QCar2_task_lane_following_probe.py
YELLOW_HI = np.array([45, 255, 255])
FX, FY, CX, CY = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
HORIZON_V = CY - FY * np.tan(PITCH)   # image row of the horizon
ROI_TOP = int(HORIZON_V) + 18         # ignore everything above (room, boxes)
FIT_MAX_FWD = 0.55                    # m from the camera (~0.61 m from the axle, as before), fit only the near part
MIN_POINTS = 40                       # pixels needed to accept a line
MAX_RMS = 0.015                       # m, line fit scatter limit
WIDTH_TOL = 0.20                      # accepted lane width error (fraction)
HEADING_MAX = 10.0                    # deg; beyond this the fit is a curve chord, not the lane at the car
CURV_MAX = 0.5                        # 1/m; tighter curves can't be extrapolated back to the axle
EDGE_BRIGHT, EDGE_DARK = 130, 110     # V levels: white surface / dark road
EDGE_MIN_RUN = 40                     # px, the white beside the edge must be at least this wide (0 = off; §39)

# -- Map-shape fit for camera steering (plan step C3, camera report §35) ---------
# vehicle_control.py (use_camera_steer) sends the map's lane curvature around
# the car's position along the path to PROFILE_PORT: "t_epoch,s,k_1,...,k_N"
# with k sampled at PROFILE_S (m from the front axle, 1/m, + = left turn). Each
# frame, the yellow centre line and the right edge are fitted SEPARATELY with
# that lane shape, leaving only the offset and the heading free (as
# c2_offline_test.py, §25/§31). Same selection rules as there.
PROFILE_S = np.round(np.arange(-0.30, 1.0001, 0.05), 3)   # keep identical in vehicle_control.py
PROFILE_PORT = 18556
PROFILE_MAX_AGE = 0.5        # s, older profiles aren't used
MF_FIT_MAX = 0.60            # m ahead of the front axle used in the fit
MF_MAX_PTS = 100             # points per boundary (subsampled, speed only; 200 gives the same result)
MF_MAX_RMS = 0.15            # cm, a fit is used below this rms
MF_MAX_DISAGREE = 3.0        # cm, yellow vs edge offset when both fit
MF_F_SCALE = 0.002           # m, robust (soft-L1) scale
MF_MAX_D, MF_MAX_H = 30.0, 60.0   # cm, deg: larger fit results are treated as diverged (§40)
_MF_S = np.arange(-0.30, 1.0, 0.008)     # lane curve samples (0.004 gives the same result)

LOG_COLUMNS = ['t_epoch', 't', 'mode', 'offset_cm', 'heading_deg', 'conf',
               'yellow_cm', 'edge_cm', 'width_cm', 'curv_1pm',
               'n_yellow', 'n_edge', 'rms_yellow_cm', 'rms_edge_cm', 'proc_ms',
               'mf_src', 'mf_d_cm', 'mf_h_deg', 'mf_rms_cm', 'mf_s', 'mf_ms',
               'sh_m', 'sh_rms0', 'sh_rms', 'sh_ok']     # along-route shift fit (§57)

MAP1, MAP2 = cv2.initUndistortRectifyMap(K, DIST, None, K, (IMG_W, IMG_H), cv2.CV_16SC2)


def to_ground(v, u):
    """Undistorted pixel rows/cols -> (forward from the front axle, left) in m, car frame, true scale."""
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
        k = 0
        while k < len(idx):
            i = idx[k]
            if EDGE_MIN_RUN:
                # a white dash in the road is a narrow bright stripe with dark road
                # after it; the road edge is followed by the wide white surface
                # (or the image border). Skip narrow stripes and look further (§39).
                dark = np.nonzero(row[i:] <= EDGE_DARK)[0]      # road-dark again after the stripe
                if len(dark) and dark[0] < EDGE_MIN_RUN:
                    k = int(np.searchsorted(idx, i + dark[0]))
                    continue
            if i >= 3 and row[max(i - 6, 0)] <= EDGE_DARK:   # must step up from dark road
                ev.append(r)
                eu.append(u0 + i)
            break
    return np.array(ev, float), np.array(eu, float)


def process(bgr):
    """One frame -> dict of measurements (log row without times) + debug data."""
    t0 = time.time()
    und = cv2.remap(bgr, MAP1, MAP2, cv2.INTER_LINEAR)
    hsv = cv2.cvtColor(und, cv2.COLOR_BGR2HSV)
    out = dict(mode='none', offset_cm=np.nan, heading_deg=np.nan, conf=0.0,
               yellow_cm=np.nan, edge_cm=np.nan, width_cm=np.nan, curv_1pm=np.nan,
               n_yellow=0, n_edge=0, rms_yellow_cm=np.nan, rms_edge_cm=np.nan)
    dbg = dict(und=und, yellow=None, edge=None, hsv=hsv)

    # Yellow line
    yv, yu, ymask = find_yellow(hsv)
    dbg['ymask'] = ymask
    dbg['yv'], dbg['yu'] = yv, yu
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
    dbg['img_fit'] = img_fit
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


def lane_from_profile(k):
    """Lane-centre curve from curvature samples at PROFILE_S, in the lane frame at
    the front axle (x along the lane). -> centre points (N,2), unit left normals (N,2)."""
    kk = np.interp(_MF_S, PROFILE_S, k)
    ds = np.diff(_MF_S, prepend=_MF_S[0])
    i0 = int(np.argmin(np.abs(_MF_S)))
    th = np.cumsum(kk * ds)
    th -= th[i0]
    x, y = np.cumsum(np.cos(th) * ds), np.cumsum(np.sin(th) * ds)
    c = np.column_stack([x - x[i0], y - y[i0]])
    n = np.column_stack([-np.sin(th), np.cos(th)])
    return c, n


def boundary_points(dbg):
    """Floor points for the map-shape fit, as in the offline scripts: the yellow line
    as one centre point per image row, the right edge searched from a line through
    all yellow pixels. -> (yellow (N,2), edge (M,2)), (forward, left) in m."""
    yv, yu, hsv = dbg['yv'], dbg['yu'], dbg['hsv']
    ypts = np.zeros((0, 2))
    if len(yv):
        cv_, cu = [], []
        for r in np.unique(yv):
            u = yu[yv == r]
            if len(u) < 3 or u.max() - u.min() > 60:   # skip rows with two separate blobs
                continue
            cv_.append(r)
            cu.append(u.mean())
        if cv_:
            f, l = to_ground(np.array(cv_), np.array(cu))
            keep = (f > 0) & (f < MF_FIT_MAX) & (l > 0)
            ypts = np.column_stack([f[keep], l[keep]])
    # Edge search starts right of the yellow line: the robust (inlier) yellow fit
    # when there is one. A line through ALL yellow pixels can be pulled away by
    # other yellow paint (near node 14 the roundabout's arc): the search then
    # started left of the divider and took the divider as the edge (§36, -27 cm).
    img_fit = dbg.get('img_fit')
    if img_fit is None and len(yv) >= MIN_POINTS:
        a, b = np.polyfit(yv, yu, 1)
        img_fit = lambda r: a * r + b
    ev, eu = find_edge(hsv, img_fit)
    epts = np.zeros((0, 2))
    if len(ev):
        f, l = to_ground(ev, eu)
        keep = (f > 0) & (f < MF_FIT_MAX)
        epts = np.column_stack([f[keep], l[keep]])
    return ypts, epts


def fit_boundary(p, c, n, side, iters=15):
    """Offset d (m, + = car left of the lane centre) and heading psi (rad) that put
    the points p (car frame) on the boundary c + side*n. Robust Gauss-Newton
    (soft-L1 weights). -> d_cm, psi_deg, rms_cm."""
    q = c + side * n
    px, py = p[:, 0], p[:, 1]
    d = psi = 0.0
    for _ in range(iters):
        cp, sp = np.cos(psi), np.sin(psi)
        pl = np.column_stack([cp * px - sp * py, sp * px + cp * py + d])
        k = np.argmin(((pl[:, None, :] - q[None, :, :]) ** 2).sum(-1), axis=1)
        nk = n[k]
        e = ((pl - q[k]) * nk).sum(1)
        J = np.column_stack([nk[:, 1], (-sp * px - cp * py) * nk[:, 0] + (cp * px - sp * py) * nk[:, 1]])
        w = 1.0 / np.sqrt(1.0 + (e / MF_F_SCALE) ** 2)
        A = J.T @ (J * w[:, None])
        try:
            step = -np.linalg.solve(A, J.T @ (w * e))
        except np.linalg.LinAlgError:
            break
        d, psi = d + step[0], psi + step[1]
        if abs(step[0]) < 1e-5 and abs(step[1]) < 1e-5:
            break
    cp, sp = np.cos(psi), np.sin(psi)
    pl = np.column_stack([cp * px - sp * py, sp * px + cp * py + d])
    k = np.argmin(((pl[:, None, :] - q[None, :, :]) ** 2).sum(-1), axis=1)
    e = ((pl - q[k]) * n[k]).sum(1)
    return 100 * d, float(np.degrees(psi)), 100 * float(np.sqrt(np.mean(e ** 2)))


SHIFT_MAX = 0.30            # m, along-route shifts tried: -SHIFT_MAX ... +SHIFT_MAX (§55-§57)
SHIFT_COARSE, SHIFT_FINE = 0.06, 0.02   # m, coarse grid, then +-2 fine steps around its best
SHIFT_MIN_DK = 0.5          # 1/m: curvature change needed within the fit window (else s is unobservable)
SHIFT_MIN_GAIN = 0.10       # rms(shift 0) - rms(best) must exceed this (same units as fit_boundary rms)
SHIFT_PERIOD = 0.2          # s, shift_fit at most this often in the live loop (~80 ms per call on the PC)


def shift_fit(dbg, k):
    """Along-route position check (camera report §55-§57): refit the map lane shape with
    the curvature profile k (at PROFILE_S around the car's s) shifted along the route,
    coarse grid then fine steps around its best. Observable only where the curvature
    changes within the view (curve starts / ends).
    -> dict(sh_m: best shift (m, + = the lane is further along than the profile's s),
            sh_rms0, sh_rms: rms at shift 0 / at the best shift, sh_ok: usable)."""
    bad = dict(sh_m=np.nan, sh_rms0=np.nan, sh_rms=np.nan, sh_ok=False)
    win = (PROFILE_S >= 0) & (PROFILE_S <= MF_FIT_MAX)
    if np.ptp(np.asarray(k)[win]) < SHIFT_MIN_DK:
        return bad
    ypts, epts = boundary_points(dbg)
    pts = []
    for p, side in ((ypts, LANE_W / 2), (epts, -LANE_W / 2)):
        if len(p) >= MIN_POINTS // 2:
            if len(p) > MF_MAX_PTS:
                p = p[np.linspace(0, len(p) - 1, MF_MAX_PTS).astype(int)]
            pts.append((p, side))
    if not pts:
        return bad
    cache = {}

    def rms_at(sh):
        sh = round(float(sh), 3)
        if sh not in cache:
            c, n = lane_from_profile(np.interp(PROFILE_S + sh, PROFILE_S, k))
            acc = m = 0
            for p, side in pts:
                dd, hh, rr = fit_boundary(p, c, n, side)
                if abs(dd) < MF_MAX_D and abs(hh) < MF_MAX_H:
                    acc += rr ** 2 * len(p)
                    m += len(p)
            cache[sh] = np.sqrt(acc / m) if m else np.inf
        return cache[sh]

    coarse = np.round(np.arange(-SHIFT_MAX, SHIFT_MAX + 1e-9, SHIFT_COARSE), 3)
    best = min(coarse, key=rms_at)
    for sh in best + SHIFT_FINE * np.array([-2, -1, 1, 2]):
        if abs(sh) <= SHIFT_MAX + 1e-9:
            rms_at(sh)
    best = min(cache, key=cache.get)
    rb, r0 = cache[best], rms_at(0.0)
    ok = (abs(best) < SHIFT_MAX - 1e-9 and np.isfinite(r0) and r0 - rb > SHIFT_MIN_GAIN
          and rb < MF_MAX_RMS)               # the best fit itself must be clean (§56: roundabout exit noise)
    return dict(sh_m=float(best), sh_rms0=float(r0), sh_rms=float(rb), sh_ok=bool(ok))


def map_fit(dbg, k):
    """Map-shape fit of one frame. -> dict(mf_src, mf_d_cm, mf_h_deg, mf_rms_cm)."""
    c, n = lane_from_profile(k)
    ypts, epts = boundary_points(dbg)
    res = {}
    for name, p, side in (('Y', ypts, LANE_W / 2), ('E', epts, -LANE_W / 2)):
        if len(p) < MIN_POINTS // 2:
            continue
        if len(p) > MF_MAX_PTS:
            p = p[np.linspace(0, len(p) - 1, MF_MAX_PTS).astype(int)]
        dd, hh, rr = fit_boundary(p, c, n, side)
        if not (abs(dd) < MF_MAX_D and abs(hh) < MF_MAX_H):
            continue                                   # diverged fit (seen: -26 cm, +425 deg, §40)
        if rr < MF_MAX_RMS:
            res[name] = (dd, hh, rr)
    pick = ('none', np.nan, np.nan, np.nan)
    if 'Y' in res and 'E' in res:
        if abs(res['Y'][0] - res['E'][0]) < MF_MAX_DISAGREE:
            pick = ('YE', (res['Y'][0] + res['E'][0]) / 2, (res['Y'][1] + res['E'][1]) / 2,
                    min(res['Y'][2], res['E'][2]))
    elif 'E' in res:
        pick = ('E', *res['E'])
    elif 'Y' in res:
        pick = ('Y', *res['Y'])
    return dict(mf_src=pick[0], mf_d_cm=pick[1], mf_h_deg=pick[2], mf_rms_cm=pick[3])


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
    ap.add_argument('--save-raw', action='store_true',
                    help='also save raw front + left frames to raw_frames/<start time>/ for offline curve tests')
    ap.add_argument('--raw-every', type=float, default=0.2, help='seconds between raw frame pairs')
    ap.add_argument('--no-shift', dest='shift', action='store_false',
                    help='do not run the along-route shift fit (camera report §57); REVERT switch')
    args = ap.parse_args()
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) if args.udp_port else None
    # Lane curvature from vehicle_control.py (camera steering, C3); only with UDP on
    psock, profile = None, None              # profile = (t_epoch, s, k array)
    if args.udp_port:
        psock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        psock.bind(('127.0.0.1', PROFILE_PORT))
        psock.setblocking(False)

    from pal.products.qcar import QCarCameras
    cams = QCarCameras(frameWidth=IMG_W, frameHeight=IMG_H, frameRate=args.rate,
                       enableFront=True, enableLeft=args.save_raw)
    raw_dir = None
    if args.save_raw:
        # New folder per run, so a recording is never overwritten
        raw_dir = os.path.join('raw_frames', time.strftime('%Y%m%d_%H%M%S'))
        os.makedirs(raw_dir)
        print(f'saving raw front + left frames to {raw_dir}/ every {args.raw_every} s')
    last_raw = 0.0
    n_raw = 0
    os.makedirs('lane_debug', exist_ok=True)
    for f in os.listdir('lane_debug'):          # don't mix images from earlier runs
        if f.endswith('.jpg'):
            os.remove(os.path.join('lane_debug', f))
    logf = open(args.log, 'w')
    logf.write(','.join(LOG_COLUMNS) + '\n')
    t_start = time.time()
    last_dbg = last_print = last_proc = last_shift = 0.0
    n = 0
    try:
        while not args.duration or time.time() - t_start < args.duration:
            # Process only new frames, at most --max-rate per second, so the
            # GPS/lidar localization isn't starved of CPU (2026-11-10 run:
            # 62 % of processed frames were repeats at ~70 Hz).
            if time.time() - last_proc < 1.0 / args.max_rate:
                time.sleep(0.005)
                continue
            # Read the cameras one by one (not readAll): with the left camera
            # on, a new left frame must not make us reprocess a stale front one.
            new_front = cams.csiFront.read()
            if raw_dir:
                cams.csiLeft.read()
            img = cams.csiFront.imageData
            if not new_front or img is None or not img.any():
                time.sleep(0.005)
                continue
            now = time.time()
            last_proc = now
            if raw_dir and now - last_raw >= args.raw_every:
                # Raw (distorted) frames, JPEG q95; file name = t_epoch, same
                # clock as lane_camera.csv and the pose= lines of vehicle_control.log
                cv2.imwrite(os.path.join(raw_dir, f'{now:.3f}_front.jpg'), img,
                            [cv2.IMWRITE_JPEG_QUALITY, 95])
                left = cams.csiLeft.imageData
                if left is not None and left.any():
                    cv2.imwrite(os.path.join(raw_dir, f'{now:.3f}_left.jpg'), left,
                                [cv2.IMWRITE_JPEG_QUALITY, 95])
                last_raw = now
                n_raw += 1
            out, dbg = process(img)
            out['t_epoch'], out['t'] = now, now - t_start
            # Map-shape fit when vehicle_control.py sends the lane curvature (C3)
            while psock:
                try:
                    vals = psock.recv(1024).decode().split(',')
                    if len(vals) == 2 + len(PROFILE_S):
                        profile = (float(vals[0]), float(vals[1]), np.array(vals[2:], float))
                except (BlockingIOError, socket.error):
                    break
                except ValueError:
                    pass
            out.update(mf_src='none', mf_d_cm=np.nan, mf_h_deg=np.nan, mf_rms_cm=np.nan,
                       mf_s=np.nan, mf_ms=np.nan, sh_m=np.nan, sh_rms0=np.nan, sh_rms=np.nan, sh_ok=0)
            if profile is not None and time.time() - profile[0] < PROFILE_MAX_AGE:
                t_mf = time.time()
                out.update(map_fit(dbg, profile[2]))
                if args.shift and now - last_shift >= SHIFT_PERIOD:
                    sf = shift_fit(dbg, profile[2])
                    if np.isfinite(sf['sh_rms0']):      # it ran (curvature change in view)
                        last_shift = now
                    out.update(sf)
                    out['sh_ok'] = int(sf['sh_ok'])
                out['mf_s'], out['mf_ms'] = profile[1], 1000 * (time.time() - t_mf)
            logf.write(','.join(fmt(out[c]) for c in LOG_COLUMNS) + '\n')
            if sock:
                msg = ','.join(fmt(out[c]) for c in ['t_epoch', 'mode', 'offset_cm', 'heading_deg', 'conf',
                                                      'mf_src', 'mf_d_cm', 'mf_h_deg', 'mf_rms_cm', 'mf_s',
                                                      'sh_m', 'sh_ok', 'sh_rms'])
                sock.sendto(msg.encode(), ('127.0.0.1', args.udp_port))
            n += 1
            if args.debug_every and now - last_dbg >= args.debug_every:
                cv2.imwrite(f"lane_debug/{out['t']:07.2f}.jpg", draw_debug(out, dbg))
                last_dbg = now
            if now - last_print >= 1.0:
                mf = ''
                if not np.isnan(out['mf_s']):
                    mf = (f"  map-fit {out['mf_src']:4s} d={out['mf_d_cm']:+5.1f} cm h={out['mf_h_deg']:+5.1f} deg "
                          f"s={out['mf_s']:+.2f} m {out['mf_ms']:.0f} ms")
                    if out['sh_ok']:
                        mf += f"  shift {out['sh_m']:+.2f} m"
                print(f"t={out['t']:6.1f}  {out['mode']:6s}  offset={out['offset_cm']:+6.1f} cm  "
                      f"heading={out['heading_deg']:+5.1f} deg  conf={out['conf']:.2f}  "
                      f"width={out['width_cm']:5.1f} cm  {out['proc_ms']:.0f} ms/frame{mf}")
                last_print = now

    except KeyboardInterrupt:
        pass
    finally:
        logf.close()
        cams.terminate()
        print(f'{n} frames logged to {args.log}')
        if raw_dir:
            print(f'{n_raw} raw frame pairs saved to {raw_dir}/')


if __name__ == '__main__':
    main()
