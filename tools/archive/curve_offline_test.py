"""
curve_offline_test.py   (runs on the PC, not the car)

Camera report §15/§19: does "visible boundary + known curvature" give the car's
lane offset on curves? Runs on the raw front frames saved by
lane_camera.py --save-raw (hand-rolled, centred pass, so the true offset ~ 0).

For each frame, the yellow pixels (left boundary) and right road edge pixels
from lane_camera.py are projected onto the floor. The boundaries are fitted
as circles of KNOWN radius around a common lane centre (radius Rc), leaving
only two unknowns: offset d (+ = car left of lane centre) and heading psi
(+ = car points left). Two models per frame:
    straight : Rc = infinity (same geometry as lane_camera.py)
    arc      : Rc = 0.891 m left turn (SDCSRoadMap outerLaneRadius, 0->2 and 2->4)
The model with the lower robust cost is reported as "best"; both are logged.

    python tools/archive/curve_offline_test.py "logs/car A/raw_frames/20261117_191853"
Output in <folder>/../curve_test_<folder name>/: curve_test.csv, offsets.png,
and bird's-eye plots of some frames (bev_<t>.png).
"""
import os
import sys
import numpy as np
import cv2
from scipy.optimize import least_squares

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
import common  # noqa: E402  qcar/, tools/ folders and ../libraries/python on sys.path
import lane_camera as lc

RC_ARC = 0.891           # m, lane-centre radius of the 0->2 and 2->4 arcs (left turns)
HALF_W = lc.LANE_W / 2   # m
FIT_MAX = 0.60           # m ahead of the front axle used in the fit (true scale since §29; ×0.724 units before)
STRAIGHT_R = 1e4         # "infinite" radius for the straight model


def boundary_points(bgr):
    """Front frame -> floor points (fwd, left) in m, true scale, for yellow and edge."""
    und = cv2.remap(bgr, lc.MAP1, lc.MAP2, cv2.INTER_LINEAR)
    hsv = cv2.cvtColor(und, cv2.COLOR_BGR2HSV)
    yv, yu, _ = lc.find_yellow(hsv)
    img_fit = None
    if len(yv) >= lc.MIN_POINTS:
        a, b = np.polyfit(yv, yu, 1)
        img_fit = lambda r: a * r + b
    ev, eu = lc.find_edge(hsv, img_fit)
    pts = {}
    for name, v, u in (('yellow', yv, yu), ('edge', ev, eu)):
        if len(v) == 0:
            pts[name] = np.zeros((0, 2))
            continue
        # lc.SCALE is 1 since the checkerboard calibration (report §29); it was
        # ×0.724 for the results in §19-§26.
        f, l = lc.to_ground(v, u)
        f = (f - lc.CAM_AHEAD) * lc.SCALE + lc.CAM_AHEAD
        l = l * lc.SCALE
        keep = (f > 0) & (f < FIT_MAX)
        if name == 'yellow':
            keep &= l > 0
        pts[name] = np.column_stack([f[keep], l[keep]])
    return pts, und


def point_errors(x, pts, rc):
    """Per boundary: distance (m) of each point from its circle. Geometry:
    lane frame (x along lane, y left), car at (0, d) with heading psi; boundary
    s (+HALF_W yellow, -HALF_W edge) is a circle of radius rc - s around (0, rc).
    In the car frame that centre is at ((rc-d) sin psi, (rc-d) cos psi)."""
    d, psi = x
    a = rc - d
    cx, cy = a * np.sin(psi), a * np.cos(psi)
    errs = []
    for name, s in (('yellow', +HALF_W), ('edge', -HALF_W)):
        p = pts[name]
        if len(p):
            errs.append(np.hypot(p[:, 0] - cx, p[:, 1] - cy) - (rc - s))
    return errs


def residuals(x, pts, rc):
    # equal weight per boundary: the yellow gives ~10x more points than the edge
    return np.concatenate([e / np.sqrt(len(e)) for e in point_errors(x, pts, rc)])


def fit(pts, rc):
    """-> offset (cm), heading (deg), rms of the point errors (cm)."""
    r = least_squares(residuals, x0=[0.0, 0.0], args=(pts, rc), loss='soft_l1', f_scale=0.002)
    e = np.concatenate(point_errors(r.x, pts, rc))
    return 100 * r.x[0], np.degrees(r.x[1]), 100 * float(np.sqrt(np.mean(e ** 2)))


# -- Piecewise model (§20): straight then arc ("entry") or arc then straight ("exit") --
S_GRID = np.arange(-0.05, 0.86, 0.05)   # m, candidate distances ahead of the axle to the change
S_SAMPLES = np.arange(-0.15, 1.0, 0.004)
MAX_YELLOW = 200                        # subsample yellow points (speed only)


def lane_curves(k1, k2, s0):
    """Lane-centre curve in the lane frame (origin at the car's foot point, x along the
    lane at the car): curvature k1 before s0, k2 after. -> centre xy, unit left normal."""
    s = S_SAMPLES
    th = np.where(s < s0, k1 * s, k1 * s0 + k2 * (s - s0))
    ds = np.diff(s, prepend=s[0])
    x, y = np.cumsum(np.cos(th) * ds), np.cumsum(np.sin(th) * ds)
    i0 = np.argmin(np.abs(s))
    c = np.column_stack([x - x[i0], y - y[i0]])
    n = np.column_stack([-np.sin(th), np.cos(th)])
    return c, n


def piece_errors(x, pts, curves):
    """Signed normal distance of each point from its boundary (centre +/- HALF_W)."""
    d, psi = x
    c, n = curves
    cp, sp = np.cos(psi), np.sin(psi)
    errs = []
    for name, s in (('yellow', +HALF_W), ('edge', -HALF_W)):
        p = pts[name]
        if not len(p):
            continue
        # car frame -> lane frame: rotate by +psi, then shift by the offset d
        pl = np.column_stack([cp * p[:, 0] - sp * p[:, 1], sp * p[:, 0] + cp * p[:, 1] + d])
        q = c + s * n
        k = np.argmin(((pl[:, None, :] - q[None, :, :]) ** 2).sum(-1), axis=1)
        errs.append(((pl - q[k]) * n[k]).sum(1))
    return errs


def fit_piecewise(pts):
    """Grid over the change distance s0 for both entry and exit; LS for (d, psi) at each.
    -> dict with the best fit: d (cm), heading (deg), rms (cm), kind, s0 (m)."""
    if len(pts['yellow']) > MAX_YELLOW:
        pts = dict(pts, yellow=pts['yellow'][np.linspace(0, len(pts['yellow']) - 1, MAX_YELLOW).astype(int)])
    best = None
    k = 1 / RC_ARC
    for kind, k1, k2 in (('entry', 0.0, k), ('exit', k, 0.0)):
        for s0 in S_GRID:
            curves = lane_curves(k1, k2, s0)
            fun = lambda x: np.concatenate([e / np.sqrt(len(e)) for e in piece_errors(x, pts, curves)])
            r = least_squares(fun, x0=[0.0, 0.0], loss='soft_l1', f_scale=0.002)
            if best is None or r.cost < best[0]:
                best = (r.cost, kind, s0, r.x, curves)
    cost, kind, s0, (d, psi), curves = best
    e = np.concatenate(piece_errors((d, psi), pts, curves))
    return dict(d=100 * d, h=np.degrees(psi), rms=100 * float(np.sqrt(np.mean(e ** 2))),
                kind=kind, s0=s0, curves=curves)


def piece_xy(fp, s):
    """Fitted piecewise boundary s in the car frame (fwd, left), for plotting."""
    c, n = fp['curves']
    q = c + s * n
    d, psi = fp['d'] / 100, np.radians(fp['h'])
    q = q - [0, d]
    f = np.cos(psi) * q[:, 0] + np.sin(psi) * q[:, 1]
    l = -np.sin(psi) * q[:, 0] + np.cos(psi) * q[:, 1]
    k = (f > -0.05) & (f < FIT_MAX + 0.2)
    return f[k], l[k]


def boundary_xy(d_cm, psi_deg, rc, s, fmax=FIT_MAX + 0.2):
    """Fitted boundary s in the car frame (fwd, left), for plotting."""
    d, psi = d_cm / 100, np.radians(psi_deg)
    if rc > 100:   # straight: l = (s - d)/cos(psi) - f tan(psi)
        f = np.linspace(0, fmax, 50)
        return f, (s - d) / np.cos(psi) - f * np.tan(psi)
    a = rc - d
    cx, cy = a * np.sin(psi), a * np.cos(psi)
    th = np.linspace(-np.pi, 0, 2000)
    f, l = cx + (rc - s) * np.cos(th), cy + (rc - s) * np.sin(th)
    k = (f > -0.05) & (f < fmax)
    return f[k], l[k]


def main():
    folder = sys.argv[1].rstrip('/\\')
    out = os.path.join(os.path.dirname(folder), 'curve_test_' + os.path.basename(folder))
    os.makedirs(out, exist_ok=True)
    frames = sorted(f for f in os.listdir(folder) if f.endswith('_front.jpg'))
    t0 = float(frames[0][:-10])
    rows = []
    for i, fn in enumerate(frames):
        t = float(fn[:-10]) - t0
        bgr = cv2.imread(os.path.join(folder, fn))
        pts, und = boundary_points(bgr)
        ny, ne = len(pts['yellow']), len(pts['edge'])
        row = dict(t=t, n_yellow=ny, n_edge=ne)
        if ny < lc.MIN_POINTS:            # same minimum point counts as lane_camera.py
            pts['yellow'] = pts['yellow'][:0]
        if ne < lc.MIN_POINTS // 2:
            pts['edge'] = pts['edge'][:0]
        if len(pts['yellow']) + len(pts['edge']) == 0:
            row.update(model='none')
            rows.append(row)
            continue
        ds, hs, rs = fit(pts, STRAIGHT_R)
        da, ha, ra = fit(pts, RC_ARC)
        best = 'arc' if ra < rs else 'straight'
        fp = fit_piecewise(pts)
        row.update(model=best, d_straight=ds, h_straight=hs, rms_straight=rs,
                   d_arc=da, h_arc=ha, rms_arc=ra,
                   d_best=da if best == 'arc' else ds,
                   d_piece=fp['d'], h_piece=fp['h'], rms_piece=fp['rms'],
                   piece_kind=fp['kind'], piece_s0=fp['s0'],
                   uses=('Y' if len(pts['yellow']) else '') + ('E' if len(pts['edge']) else ''))
        rows.append(row)
        if i % 5 == 0 or 12.8 < t < 28.5:   # bird's-eye plot every ~1 s, every frame while moving
            save_bev(out, t, pts, row, und, fp)

    cols = ['t', 'model', 'uses', 'n_yellow', 'n_edge', 'd_best', 'd_straight', 'h_straight',
            'rms_straight', 'd_arc', 'h_arc', 'rms_arc',
            'd_piece', 'h_piece', 'rms_piece', 'piece_kind', 'piece_s0']
    text = ('model', 'uses', 'piece_kind')
    with open(os.path.join(out, 'curve_test.csv'), 'w') as f:
        f.write(','.join(cols) + '\n')
        for r in rows:
            f.write(','.join(lc.fmt(r.get(c, np.nan)) if c not in text
                             else str(r.get(c, '')) for c in cols) + '\n')
    plot_offsets(out, rows)
    print(f'{len(rows)} frames -> {out}')


def save_bev(out, t, pts, row, und, fp):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, (ax0, ax1) = plt.subplots(1, 2, figsize=(11, 4.2))
    ax0.imshow(cv2.cvtColor(und, cv2.COLOR_BGR2RGB))
    ax0.set_title(f't={t:.1f} s (undistorted)')
    ax0.axis('off')
    for name, col in (('yellow', 'orange'), ('edge', 'blue')):
        p = pts[name]
        if len(p):
            ax1.plot(-p[:, 1] * 100, p[:, 0] * 100, '.', ms=2, color=col, label=name)
    for model, rc, ls in (('straight', STRAIGHT_R, '--'), ('arc', RC_ARC, '-')):
        d, h = row[f'd_{model}'], row[f'h_{model}']
        for s in (+HALF_W, -HALF_W):
            f, l = boundary_xy(d, h, rc, s)
            ax1.plot(-l * 100, f * 100, ls, color='k' if model == 'arc' else 'grey', lw=1,
                     label=f'{model}: d={d:+.1f} cm, rms={row[f"rms_{model}"]:.2f}' if s > 0 else None)
    for s in (+HALF_W, -HALF_W):
        f, l = piece_xy(fp, s)
        ax1.plot(-l * 100, f * 100, '-', color='r', lw=1.2,
                 label=f'piecewise {fp["kind"]} s0={100 * fp["s0"]:.0f} cm: d={fp["d"]:+.1f} cm, '
                       f'rms={fp["rms"]:.2f}' if s > 0 else None)
    ax1.plot(0, 0, 'r^', ms=8)
    ax1.set_xlabel('right of front axle centre (cm)')
    ax1.set_ylabel('ahead of front axle (cm)')
    ax1.set_xlim(-40, 40)
    ax1.set_ylim(-5, 85)
    ax1.set_aspect('equal')
    ax1.legend(fontsize=7, loc='upper right')
    ax1.set_title(f'piecewise: d={fp["d"]:+.1f} cm, heading {fp["h"]:+.1f} deg')
    fig.tight_layout()
    fig.savefig(os.path.join(out, f'bev_{t:05.1f}.png'), dpi=80)
    plt.close(fig)


def plot_offsets(out, rows):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    t = np.array([r['t'] for r in rows])
    g = lambda k: np.array([r.get(k, np.nan) for r in rows], float)
    fig, ax = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
    ax[0].plot(t, g('d_straight'), '.', color='grey', label='straight model')
    ax[0].plot(t, g('d_arc'), '.', color='k', label='arc model (R=0.891 m)')
    ax[0].plot(t, g('d_best'), '-', color='orange', lw=0.8, label='best of straight/arc')
    ax[0].plot(t, g('d_piece'), '-', color='r', lw=1.2, label='piecewise (s0 fitted)')
    ax[0].axhline(0, color='g', lw=0.8)
    ax[0].set_ylim(-20, 20)
    ax[0].set_ylabel('offset d (cm, + = left)')
    ax[0].legend(fontsize=8)
    ax[1].plot(t, g('rms_straight'), '.', color='grey', label='straight')
    ax[1].plot(t, g('rms_arc'), '.', color='k', label='arc')
    ax[1].plot(t, g('rms_piece'), '-', color='r', lw=1, label='piecewise')
    ax[1].set_ylabel('fit rms (cm)')
    ax[1].set_xlabel('t (s)')
    ax[1].set_ylim(0, 3)
    ax[1].legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(os.path.join(out, 'offsets.png'), dpi=90)
    plt.close(fig)


if __name__ == '__main__':
    main()
