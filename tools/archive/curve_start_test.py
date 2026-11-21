"""
curve_start_test.py   (runs on the PC, not the car)

Plan step C1 (camera report Â§23): can the front camera see where a curve
starts ("entry", straight -> arc) or ends ("exit", arc -> straight) while that
point is still AHEAD, inside the visible range? If yes, the wheel encoder can
carry it from there to the axle, and the curve fit doesn't need the GPS.

For each front frame, the yellow line and the right edge are fitted SEPARATELY
(the edge isn't a lane boundary at intersections, Â§20) with four lane shapes:
    straight, arc (0.891 m left), entry at s0, exit at s0,
with s0 limited to the visible range S_VISIBLE. A change point counts as seen
when the best entry/exit fit is clearly better than both pure shapes.

    python tools/archive/curve_start_test.py "logs/car A/raw_frames/20261117_191853" 13 28.5
Output: <folder>/../curve_start_<folder name>.csv, printed table.
"""
import os
import sys
import numpy as np
import cv2
from scipy.optimize import least_squares

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
import common  # noqa: E402  qcar/, tools/ folders and ../libraries/python on sys.path
import lane_camera as lc
import curve_offline_test as cot

S_VISIBLE = np.arange(0.25, 0.66, 0.025)   # m ahead of the axle (true scale since §29; ×0.724 units before)
CLEAR = 0.5                                # change seen if rms < CLEAR * best pure rms
K = 1 / cot.RC_ARC


def yellow_centreline(bgr):
    """Yellow line as one point per image row (centre of the mask pixels in that row),
    projected like cot.boundary_points. The line is 1-2 cm wide, so all-pixel fits
    have ~0.4 cm scatter that hides the change point."""
    und = cv2.remap(bgr, lc.MAP1, lc.MAP2, cv2.INTER_LINEAR)
    hsv = cv2.cvtColor(und, cv2.COLOR_BGR2HSV)
    vs, us, _ = lc.find_yellow(hsv)
    if len(vs) == 0:
        return np.zeros((0, 2))
    rows = np.unique(vs)
    cv_, cu = [], []
    for r in rows:
        u = us[vs == r]
        if len(u) < 3 or u.max() - u.min() > 60:   # skip rows with two separate blobs
            continue
        cv_.append(r)
        cu.append(u.mean())
    if not cv_:
        return np.zeros((0, 2))
    # lc.SCALE is 1 since the checkerboard calibration (§29); ×0.724 for §23-§26
    f, l = lc.to_ground(np.array(cv_), np.array(cu))
    f = (f - lc.CAM_AHEAD) * lc.SCALE + lc.CAM_AHEAD
    l = l * lc.SCALE
    keep = (f > 0) & (f < cot.FIT_MAX) & (l > 0)
    return np.column_stack([f[keep], l[keep]])


def fit_shape(pts, k1, k2, s0):
    curves = cot.lane_curves(k1, k2, s0)
    fun = lambda x: np.concatenate([e / np.sqrt(len(e)) for e in cot.piece_errors(x, pts, curves)])
    r = least_squares(fun, x0=[0.0, 0.0], loss='soft_l1', f_scale=0.002)
    e = np.concatenate(cot.piece_errors(r.x, pts, curves))
    return 100 * float(np.sqrt(np.mean(e ** 2))), 100 * r.x[0], np.degrees(r.x[1])


def analyse(pts):
    """-> dict: best pure shape and best change point (rms, d, h, s0)."""
    straight = fit_shape(pts, 0, 0, 0)
    arc = fit_shape(pts, K, K, 0)
    pure = ('straight', *straight) if straight[0] <= arc[0] else ('arc', *arc)
    best = None
    for kind, k1, k2 in (('entry', 0, K), ('exit', K, 0)):
        for s0 in S_VISIBLE:
            r = fit_shape(pts, k1, k2, s0)
            if best is None or r[0] < best[1]:
                best = (kind, *r, s0)
    seen = best[1] < CLEAR * pure[1]
    return dict(pure=pure, change=best, seen=seen)


def main():
    folder = sys.argv[1].rstrip('/\\')
    t_from, t_to = float(sys.argv[2]), float(sys.argv[3])
    frames = sorted(f for f in os.listdir(folder) if f.endswith('_front.jpg'))
    t0 = float(frames[0][:-10])
    out = os.path.join(os.path.dirname(folder), 'curve_start_' + os.path.basename(folder) + '.csv')
    lines = ['t,boundary,n,pure,pure_rms,pure_d,pure_h,change,change_rms,change_d,change_h,s0,seen']
    print(f"{'t':>5}  {'yellow':52s}  edge")
    for fn in frames:
        t = float(fn[:-10]) - t0
        if not t_from <= t <= t_to:
            continue
        bgr = cv2.imread(os.path.join(folder, fn))
        pts, _ = cot.boundary_points(bgr)
        pts['yellow'] = yellow_centreline(bgr)
        txt = []
        for name, nmin in (('yellow', lc.MIN_POINTS // 2), ('edge', lc.MIN_POINTS // 2)):
            p = pts[name]
            if len(p) < nmin:
                txt.append(f"{'-':52s}")
                continue
            if len(p) > cot.MAX_YELLOW:
                p = p[np.linspace(0, len(p) - 1, cot.MAX_YELLOW).astype(int)]
            one = {'yellow': p if name == 'yellow' else p[:0], 'edge': p if name == 'edge' else p[:0]}
            r = analyse(one)
            pk, prms, pd, ph = r['pure']
            ck, crms, cd, ch, s0 = r['change']
            lines.append(f"{t:.2f},{name},{len(p)},{pk},{prms:.3f},{pd:.2f},{ph:.2f},{ck},{crms:.3f},{cd:.2f},{ch:.2f},{s0:.3f},{int(r['seen'])}")
            mark = f"SEEN {ck} s0={100 * s0:2.0f}cm d={cd:+5.1f} h={ch:+5.1f} rms={crms:.2f}" if r['seen'] else ''
            txt.append(f"{pk[:3]} d={pd:+5.1f} h={ph:+5.1f} rms={prms:.2f} {mark:24s}"[:52].ljust(52) if not mark
                       else f"{mark} (pure {pk[:3]} {prms:.2f})"[:52].ljust(52))
        print(f"{t:5.1f}  {txt[0]}  {txt[1]}")
    with open(out, 'w') as f:
        f.write('\n'.join(lines) + '\n')
    print('->', out)


if __name__ == '__main__':
    main()
