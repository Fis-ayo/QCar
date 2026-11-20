"""
road_check.py  (PC only)

Camera report §29: does the checkerboard calibration (§28, calib_solve.py) give a
floor projection that agrees with the road? Three checks, the ones that
contradicted each other in §26 with the borrowed intrinsics:

  1. Strip image (logs/car A/forward_check_front.png, paper strips at 30/45/60 cm
     from the front axle): pitch from the lane lines' vanishing point, camera
     height from the lane width (25.7 cm), camera position ahead of the axle
     from the strips.
  2. Lane width: yellow and edge must be parallel and 25.7 cm apart, no ×0.724.
  3. Curves: the C2 pass (§25, hand-rolled, centred) fitted with the map's lane
     shape at the encoder position, for a range of position shifts, old (×0.724)
     vs new projection. The expected heading at the front axle is NOT 0 on an arc:
     the turn centre is abeam the rear axle, so a car tracking the lane points
     th(s - L) - th(s) relative to the lane at the front axle (about -16 deg on
     the 0.891 m arcs). Counts clean edge fits whose heading is within 3 deg of
     that, and how well the yellow and edge offsets agree.

    python qcar/road_check.py
    python qcar/road_check.py --skip-curves
"""
import argparse
import json
import os
import sys
import numpy as np
import cv2
from scipy.optimize import least_squares

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, '..', '..', 'libraries', 'python'))
import lane_camera as lc

LOGS = os.path.join(HERE, '..', 'logs', 'car A')
CALIB = os.path.join(LOGS, 'calib_frames', 'calib_result_20261118_203057+20261118_204943', 'calib_result.json')
STRIP_IMG = os.path.join(LOGS, 'forward_check_front.png')
STRIP_TRUE = np.array([0.30, 0.45, 0.60])   # m from the front axle (user's ruler)
C2_FRAMES = os.path.join(LOGS, 'raw_frames', '20261117_204844')
C2_ODO = os.path.join(LOGS, 'odometry.csv')
WHEELBASE = 0.256                          # m
SHIFTS = np.round(np.arange(-0.10, 0.61, 0.05), 2)   # m, position shift (§25 best: +0.35 with ×0.724)


def load_calib():
    r = json.load(open(CALIB))['standard']
    return np.array(r['K']), np.array(r['DIST'])


def ground(v, u, K, h, p, a):
    """Undistorted pixel (row v, col u) -> (forward from the front axle, left), m."""
    x = (u - K[0, 2]) / K[0, 0]
    y = (v - K[1, 2]) / K[1, 1]
    t = h / (np.sin(p) + y * np.cos(p))
    return t * (np.cos(p) - y * np.sin(p)) + a, -t * x


def robust_polyfit(x, y, iters=3, floor=2.0):
    a, b = np.polyfit(x, y, 1)
    m = np.ones(len(x), bool)
    for _ in range(iters):
        r = np.abs(y - (a * x + b))
        m = r < max(floor, 2.5 * np.median(r))
        a, b = np.polyfit(x[m], y[m], 1)
    return a, b, m


def strip_rows(und, u, v_lo=240, v_hi=400, thr=140):
    """Centre rows of the bright strips in a 5-pixel column at u."""
    V = cv2.cvtColor(und, cv2.COLOR_BGR2HSV)[:, :, 2].astype(int)
    col = V[v_lo:v_hi, u - 2:u + 3].mean(1) > thr
    rows, start = [], None
    for i, b in enumerate(np.r_[col, False]):
        if b and start is None:
            start = i
        elif not b and start is not None:
            rows.append(v_lo + (start + i - 1) / 2)
            start = None
    return np.array(rows)


def strip_check(K, D):
    """Checks 1 and 2. -> (h, pitch, ahead) for the new projection."""
    print('== 1+2. strip image with the new intrinsics')
    m1, m2 = cv2.initUndistortRectifyMap(K, D, None, K, (lc.IMG_W, lc.IMG_H), cv2.CV_16SC2)
    und = cv2.remap(cv2.imread(STRIP_IMG), m1, m2, cv2.INTER_LINEAR)
    hsv = cv2.cvtColor(und, cv2.COLOR_BGR2HSV)
    lc.ROI_TOP, lc.CX = 215, K[0, 2]          # strip image: start below the far strip
    yv, yu, _ = lc.find_yellow(hsv)
    k = yv > 240
    ay, by = np.polyfit(yv[k], yu[k], 1)
    ev, eu = lc.find_edge(hsv, lambda r: ay * r + by)
    ae, be, m = robust_polyfit(ev, eu)
    v_vp = (be - by) / (ay - ae)
    pitch = np.arctan((K[1, 2] - v_vp) / K[1, 1])
    print(f'  yellow {k.sum()} px, edge {m.sum()}/{len(ev)} rows; vanishing point row {v_vp:.1f} '
          f'(CY {K[1, 2]:.1f}) -> pitch {np.degrees(pitch):+.2f} deg')

    rows = np.arange(250, 330, 10.0)

    def lane(h, p):
        fy, ly = ground(rows, ay * rows + by, K, h, p, 0)
        fe, le = ground(rows, ae * rows + be, K, h, p, 0)
        py, pe = np.polyfit(fy, ly, 1), np.polyfit(fe, le, 1)
        f = np.linspace(0.25, 0.60, 8)
        w = (np.polyval(py, f) - np.polyval(pe, f)) * np.cos(np.arctan((py[0] + pe[0]) / 2))
        return w, np.degrees(np.arctan(py[0])), np.degrees(np.arctan(pe[0]))

    w1, hy, he = lane(1.0, pitch)
    h = lc.LANE_W / w1.mean()
    w, _, _ = lane(h, pitch)
    print(f'  lane lines: yellow {hy:+.1f} deg, edge {he:+.1f} deg (parallel if equal); '
          f'width {100 * w.min():.1f}-{100 * w.max():.1f} cm over 25-60 cm ahead')
    print(f'  camera height from the lane width: {100 * h:.1f} cm (ruler: 11 cm to the top, lens ~1 cm lower)')

    sv = strip_rows(und, int(round(K[0, 2])))
    if len(sv) != 3:
        print(f'  strip rows found: {sv} (expected 3), strip check skipped')
        return h, pitch, 0.06
    sv = np.sort(sv)[::-1]                        # nearest first
    f0, _ = ground(sv, K[0, 2], K, h, pitch, 0)
    a = float(np.mean(STRIP_TRUE - f0))
    print(f'  strip rows {sv}; spacing {100 * (f0[1] - f0[0]):.1f}, {100 * (f0[2] - f0[1]):.1f} cm (ruler 15, 15)')
    print(f'  camera {100 * a:.1f} cm ahead of the axle -> strips at '
          + ' / '.join(f'{100 * x:.1f}' for x in f0 + a) + ' cm (ruler 30 / 45 / 60)')
    # 1 px in the image at each strip, for scale
    f1, _ = ground(sv + 1, K[0, 2], K, h, pitch, 0)
    print('  1 image row there = ' + ' / '.join(f'{100 * abs(x):.1f}' for x in f0 - f1) + ' cm')

    def res(x):
        f, _ = ground(sv, K[0, 2], K, x[0], x[1], x[2])
        ww, _, _ = lane(x[0], x[1])
        return np.r_[(f - STRIP_TRUE) * 100, (ww - lc.LANE_W) * 100]
    s = least_squares(res, [h, pitch, a])
    print(f'  free fit (h, pitch, ahead): {100 * s.x[0]:.1f} cm, {np.degrees(s.x[1]):+.2f} deg, '
          f'{100 * s.x[2]:.1f} cm; worst residual {np.abs(s.fun).max():.1f} cm')
    return h, pitch, a


# The projection lane_camera.py used until 2026-11-18 (borrowed intrinsics, ×0.724).
K_OLD = np.array([[495.84, 0.00, 408.03], [0.00, 454.60, 181.21], [0.00, 0.00, 1.00]])
DIST_OLD = np.array([-0.57513, 0.37175, -0.00489, -0.00277, -0.11136])
OLD = dict(h=0.11, pitch=np.radians(-2.67), ahead=0.10, scale=0.257 / 0.355)


def set_projection(K, D, h, pitch, ahead, scale=1.0):
    """Point lane_camera's projection globals at a calibration."""
    lc.K, lc.DIST = K, D
    lc.FX, lc.FY, lc.CX, lc.CY = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    lc.PITCH, lc.CAM_H, lc.CAM_AHEAD, lc.SCALE = pitch, h, ahead, scale
    lc.HORIZON_V = lc.CY - lc.FY * np.tan(pitch)
    lc.ROI_TOP = int(lc.HORIZON_V) + 18
    lc.MAP1, lc.MAP2 = cv2.initUndistortRectifyMap(K, D, None, K, (lc.IMG_W, lc.IMG_H), cv2.CV_16SC2)


def curve_check(label):
    import curve_offline_test as cot
    import curve_start_test as cst
    import c2_offline_test as c2
    path = c2.load_path()
    odo = np.genfromtxt(C2_ODO, delimiter=',', names=True)
    moving = odo['dist_m'] > 0.02
    th_car = np.radians(-90 + odo['heading_deg'][moving])
    cand = np.arange(-1.2, 0.0, 0.005)
    cost = [np.mean((np.interp(c + odo['dist_m'][moving], path['s'], path['th']) - th_car) ** 2) for c in cand]
    s_ref0 = cand[int(np.argmin(cost))]

    frames = []
    for fn in sorted(f for f in os.listdir(C2_FRAMES) if f.endswith('_front.jpg')):
        t = float(fn[:-10])
        if not (odo['t_epoch'][0] <= t <= odo['t_epoch'][-1]):
            continue
        dist = float(np.interp(t, odo['t_epoch'], odo['dist_m']))
        if not 0.02 < dist < odo['dist_m'][-1] - 0.02:
            continue
        bgr = cv2.imread(os.path.join(C2_FRAMES, fn))
        pts, _ = cot.boundary_points(bgr)
        frames.append((dist, cst.yellow_centreline(bgr), pts['edge']))

    print(f'\n== 3. curves, {label}: {len(frames)} moving frames of the C2 pass')
    print('  shift  clean edge  |h-h_exp|<3deg  median|h-h_exp|  median d(edge)  Y-E agree (n, median |dY-dE|)')
    best = None
    for sh in SHIFTS:
        n_ok, n_clean, hs, ds, dis = 0, 0, [], [], []
        for dist, yp, ep in frames:
            s_ax = s_ref0 + 0.13 + sh + dist
            curves = c2.local_curves(path, s_ax)
            h_exp = np.degrees(np.interp(s_ax - WHEELBASE, path['s'], path['th'])
                               - np.interp(s_ax, path['s'], path['th']))
            r = {}
            for name, p in (('Y', yp), ('E', ep)):
                if len(p) < lc.MIN_POINTS // 2:
                    continue
                if len(p) > cot.MAX_YELLOW:
                    p = p[np.linspace(0, len(p) - 1, cot.MAX_YELLOW).astype(int)]
                one = {'yellow': p if name == 'Y' else p[:0], 'edge': p if name == 'E' else p[:0]}
                dd, hh, rr = c2.fit_dh(one, curves)
                if rr < c2.MAX_RMS:
                    r[name] = (dd, hh)
            if 'E' in r:
                n_clean += 1
                hs.append(abs(r['E'][1] - h_exp))
                ds.append(r['E'][0])
                n_ok += abs(r['E'][1] - h_exp) < 3
            if 'Y' in r and 'E' in r:
                dis.append(abs(r['Y'][0] - r['E'][0]))
        mh = np.median(hs) if hs else np.nan
        print(f'  {sh:+.2f}   {n_clean:4d}        {n_ok:4d}            {mh:5.1f}          {np.median(ds) if ds else np.nan:+6.1f} cm'
              f'       {len(dis):3d}, {np.median(dis) if dis else np.nan:4.1f} cm')
        if best is None or n_ok > best[1]:
            best = (sh, n_ok)
    print(f'  best shift {best[0]:+.2f} m ({best[1]} clean edge fits with the expected heading +-3 deg)')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--skip-curves', action='store_true')
    args = ap.parse_args()
    K, D = load_calib()
    print(f'calibration: FX {K[0, 0]:.1f} FY {K[1, 1]:.1f} CX {K[0, 2]:.1f} CY {K[1, 2]:.1f}')
    h, pitch, ahead = strip_check(K, D)
    if args.skip_curves:
        return
    set_projection(K_OLD, DIST_OLD, OLD['h'], OLD['pitch'], OLD['ahead'], OLD['scale'])
    curve_check('old projection (borrowed intrinsics, x0.724)')
    set_projection(K, D, h, pitch, ahead)
    curve_check(f'new projection (h {100 * h:.1f} cm, pitch {np.degrees(pitch):+.2f} deg, ahead {100 * ahead:.1f} cm)')


if __name__ == '__main__':
    main()
