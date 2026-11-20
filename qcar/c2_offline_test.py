"""
c2_offline_test.py   (runs on the PC, not the car)

Plan step C2 (camera report §25): emulate, offline, how the car would get its
lane offset without the GPS table, on a hand-rolled centred pass (true offset ~ 0):

  * position along the path  s = s_start + encoder distance (odometry_log.py);
    s_start is corrected whenever the camera sees a curve end (arc -> straight).
  * lane shape at each frame from the map path at s (any curvature profile).
  * camera offset/heading from the yellow centre line and the right edge, each
    fitted with that shape; used when the fit is clean.
  * otherwise (intersections, curve starts) dead reckoning from the last camera
    value with the encoder distance and the gyro heading.

    python qcar/c2_offline_test.py "logs/car A/raw_frames/20261117_204844" "logs/car A/odometry.csv" [shift_m] [mode] [enc]
      shift_m  test: error added to the start position (m)
      enc      optional encoder distance scale (default 1.0; 0.98 for car A, report §34)
      mode     gyro (default): position corrected by matching the gyro heading to the map
               cam:   position corrected by camera curve ends (§25, unreliable)
               nofix: no position correction
Output next to the frames folder: c2_<folder>_fx<FX>[_shift..]_<mode>.csv/.png (FX = calibration in use)

2026-11-18 (report §31), after the checkerboard calibration (§28-§30):
  * expected heading. The turn centre is level with the REAR axle, so a car that
    follows the lane points h_exp = th(s - L) - th(s) relative to the lane at
    the front axle (about -16 deg on the 0.891 m arcs, 0 on straights). The
    camera heading is accepted when it is within MAX_HEAD of h_exp (was: of 0).
  * dead reckoning. The offset changes with the heading ERROR h - h_exp (the
    body moves along the lane when h = h_exp), not with h itself.
  * start position. The gyro/map alignment matches the body heading, i.e. the
    lane direction at the rear axle, so front axle = reference + L (was + L/2).

2026-11-18 (report §32), position along the path ('gyro' mode):
  * the encoder moves s; the body heading in map coordinates is gyro + g_off.
    g_off is learned from the camera heading on a straight: while parked at the
    start with a plain straight-line fit (needs no position), and while driving on
    mapped straights (body = lane + h there).
  * at every frame, the last GYRO_WIN m of encoder/gyro samples are compared with
    the map direction at the REAR axle (the body follows it) for position errors
    in +-GYRO_SEARCH; used only when the map turns by > GYRO_MIN_TURN inside the
    window. s moves towards the best match by GYRO_GAIN of it, at most GYRO_STEP.
  * camera curve ends are only checked (gated near a mapped curve end), not applied.
"""
import os
import sys
import numpy as np
import cv2
from scipy.optimize import least_squares

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'libraries', 'python'))
import lane_camera as lc
import curve_offline_test as cot
import curve_start_test as cst

ROUTE = [12, 0, 2, 4, 14]    # path for the pass (start on 12 -> 0, before node 0; 4 -> 14 so the fit window never runs off the end)
MAX_RMS = 0.15               # cm, a camera OFFSET is used below this fit rms
HEAD_RMS = 0.10              # cm, the camera HEADING is used only below this (else gyro)
MAX_HEAD = 8.0               # deg, camera heading beyond this isn't trusted
WHEELBASE = 0.256            # m
MAX_DISAGREE = 3.0           # cm, yellow vs edge offset when both are fitted
GYRO_WIN = 0.5               # m of encoder/gyro history matched against the map
GYRO_SEARCH = 0.40           # m, +- position error searched
GYRO_MIN_TURN = np.radians(15)   # the map must turn this much inside the window
GYRO_GAIN = 0.3              # fraction of the best match applied per frame
GYRO_STEP = 0.03             # m, largest correction per frame
_SS = np.arange(-0.15, 1.0, 0.004)
STRAIGHT = (np.column_stack([_SS, np.zeros_like(_SS)]), np.tile([0.0, 1.0], (len(_SS), 1)))  # straight lane
EXIT_GATE = (0.20, 0.60)    # m ahead: a mapped curve end must be here for a camera check
EXIT_SEARCH = (0.0, 1.0)     # m ahead of the estimate: look for a mapped curve end here
                             # (wide, so a position error of up to ~0.4 m is still caught)


# -- Map path -----------------------------------------------------------------
def load_path():
    from hal.products.mats import SDCSRoadMap
    rm = SDCSRoadMap(leftHandTraffic=False)
    p = rm.generate_path(ROUTE)
    xy = p[:2].T
    keep = np.r_[True, np.hypot(*np.diff(xy, axis=0).T) > 1e-6]
    xy = xy[keep]
    seg = np.hypot(*np.diff(xy, axis=0).T)
    s = np.r_[0, np.cumsum(seg)]
    th = np.unwrap(np.arctan2(np.gradient(xy[:, 1]), np.gradient(xy[:, 0])))
    k = np.gradient(th, s)
    node0 = np.array(rm.get_node_pose(0)).squeeze()[:2]
    s_node0 = s[np.argmin(np.hypot(*(xy - node0).T))]
    return dict(s=s - s_node0, xy=xy, th=th, k=k)


def local_curves(path, s_axle):
    """Lane centre around s_axle in the lane frame at s_axle (x along the lane)."""
    s = path['s']
    m = (s > s_axle - 0.15) & (s < s_axle + 1.0)
    i0 = np.argmin(np.abs(s - s_axle))
    th0 = path['th'][i0]
    R = np.array([[np.cos(th0), np.sin(th0)], [-np.sin(th0), np.cos(th0)]])
    c = (path['xy'][m] - path['xy'][i0]) @ R.T
    th = path['th'][m] - th0
    n = np.column_stack([-np.sin(th), np.cos(th)])
    return c, n


def fit_dh(pts, curves):
    fun = lambda x: np.concatenate([e / np.sqrt(len(e)) for e in cot.piece_errors(x, pts, curves)])
    r = least_squares(fun, x0=[0.0, 0.0], loss='soft_l1', f_scale=0.002)
    e = np.concatenate(cot.piece_errors(r.x, pts, curves))
    return 100 * r.x[0], np.degrees(r.x[1]), 100 * float(np.sqrt(np.mean(e ** 2)))


def next_exit(path, s_axle):
    """Distance ahead of s_axle to the next mapped arc -> straight change in EXIT_RANGE, or None."""
    s, k = path['s'], np.abs(path['k'])
    m = (s > s_axle + EXIT_SEARCH[0]) & (s < s_axle + EXIT_SEARCH[1])
    idx = np.nonzero(m)[0]
    for a, b in zip(idx[:-1], idx[1:]):
        if k[a] > 0.5 and k[b] <= 0.5:
            return s[b] - s_axle
    return None


# -- Camera points (cached) ---------------------------------------------------
def frame_points(folder):
    # cache per calibration (FX in the name): the points depend on the projection
    cache = os.path.join(os.path.dirname(folder),
                         f'c2_points_{os.path.basename(folder)}_fx{lc.FX:.0f}.npz')
    if os.path.exists(cache):
        z = np.load(cache, allow_pickle=True)
        return list(z['t']), list(z['y']), list(z['e'])
    ts, ys, es = [], [], []
    for fn in sorted(f for f in os.listdir(folder) if f.endswith('_front.jpg')):
        bgr = cv2.imread(os.path.join(folder, fn))
        pts, _ = cot.boundary_points(bgr)
        ts.append(float(fn[:-10]))
        ys.append(cst.yellow_centreline(bgr))
        es.append(pts['edge'])
    np.savez(cache, t=ts, y=np.array(ys, dtype=object), e=np.array(es, dtype=object))
    return ts, ys, es


def gyro_match(path, s_rear, th_body):
    """Position error (m) that best lines up the body heading samples with the map
    direction at the rear axle, or None if the map hardly turns in the window."""
    th_map = np.interp(s_rear, path['s'], path['th'])
    if np.ptp(np.interp(np.linspace(s_rear.min() - GYRO_SEARCH, s_rear.max() + GYRO_SEARCH, 50),
                        path['s'], path['th'])) < GYRO_MIN_TURN:
        return None
    if np.ptp(th_map) < GYRO_MIN_TURN / 2:
        return None
    cand = np.arange(-GYRO_SEARCH, GYRO_SEARCH + 1e-9, 0.0025)
    cost = [np.mean(np.angle(np.exp(1j * (th_body - np.interp(s_rear + c, path['s'], path['th'])))) ** 2)
            for c in cand]
    i = int(np.argmin(cost))
    if i in (0, len(cand) - 1):          # at the edge of the search: not trusted
        return None
    return float(cand[i])


def main():
    folder, odo_file = sys.argv[1].rstrip('/\\'), sys.argv[2]
    s_start_err = float(sys.argv[3]) if len(sys.argv) > 3 else 0.0   # test: shift of the initial position (m)
    mode = sys.argv[4] if len(sys.argv) > 4 else 'gyro'
    enc_scale = float(sys.argv[5]) if len(sys.argv) > 5 else 1.0     # encoder distance scale (§34: 0.98 on car A)
    assert mode in ('gyro', 'cam', 'nofix'), mode
    use_fixes = mode == "cam"
    path = load_path()
    odo = np.genfromtxt(odo_file, delimiter=',', names=True)
    odo['dist_m'] *= enc_scale
    ts, ys, es = frame_points(folder)

    # Reference only: the gyro heading profile vs the map gives where the car started
    # (stands in for the coarse GPS position at the start).
    moving = odo['dist_m'] > 0.02
    dist_o = odo['dist_m'][moving]
    th_car = np.radians(-90 + odo['heading_deg'][moving])
    cand = np.arange(-1.2, 0.0, 0.005)
    cost = [np.mean((np.interp(c + dist_o, path['s'], path['th']) - th_car) ** 2) for c in cand]
    s_ref0 = cand[int(np.argmin(cost))]
    print(f'gyro/map alignment: car reference point at s = {s_ref0:+.3f} m at the start (node 0 = 0)')

    s0_true = s_ref0 + WHEELBASE                # best estimate of the truth (§31: shift 0 is best)
    s0_est = s0_true + s_start_err              # front axle = reference (rear axle) + L
    rows, exit_fixes, exit_checks = [], [], []
    g_off, g_n = None, 0                         # body heading = gyro + g_off (learned on straights)
    d = h = 0.0
    prev = None
    for t, yp, ep in zip(ts, ys, es):
        if t < odo['t_epoch'][0] or t > odo['t_epoch'][-1]:
            continue
        dist = float(np.interp(t, odo['t_epoch'], odo['dist_m']))
        gyro = np.radians(float(np.interp(t, odo['t_epoch'], odo['heading_deg'])))
        # position along the path from the gyro heading vs the map (rear axle)
        gfix = None
        if mode == 'gyro' and g_off is not None:
            w = (odo['t_epoch'] <= t) & (odo['dist_m'] > dist - GYRO_WIN) & (odo['dist_m'] > 0.02)
            if w.sum() > 10 and np.ptp(odo['dist_m'][w]) > 0.6 * GYRO_WIN:
                s_rear = s0_est + odo['dist_m'][w] - WHEELBASE
                gfix = gyro_match(path, s_rear, np.radians(odo['heading_deg'][w]) + g_off)
                if gfix is not None:
                    s0_est += float(np.clip(GYRO_GAIN * gfix, -GYRO_STEP, GYRO_STEP))
        s_axle = s0_est + dist
        th_path = float(np.interp(s_axle, path['s'], path['th']))
        h_exp = np.degrees(float(np.interp(s_axle - WHEELBASE, path['s'], path['th'])) - th_path)
        # dead-reckoning prediction from the previous frame
        if prev is not None:
            ds = dist - prev['dist']
            d += 100 * np.sin(np.radians(h - prev['h_exp'])) * ds
            h += np.degrees((gyro - prev['gyro']) - (th_path - prev['th_path']))
        curves = local_curves(path, s_axle)
        res = {}
        for name, p, nmin in (('Y', yp, lc.MIN_POINTS // 2), ('E', ep, lc.MIN_POINTS // 2)):
            if len(p) < nmin:
                continue
            if len(p) > cot.MAX_YELLOW:
                p = p[np.linspace(0, len(p) - 1, cot.MAX_YELLOW).astype(int)]
            one = {'yellow': p if name == 'Y' else p[:0], 'edge': p if name == 'E' else p[:0]}
            dd, hh, rr = fit_dh(one, curves)
            if rr < MAX_RMS:
                res[name] = (dd, hh, rr)
            # curve end seen? (edge only, as in §24). The camera gives the distance
            # to it (s0, 25-60 cm); the map says where it is -> position correction.
            if name == 'E' and not use_fixes:
                # gated check only: a mapped curve end must be 20-60 cm ahead
                ahead = next_exit(path, s_axle)
                if ahead is not None and EXIT_GATE[0] < ahead < EXIT_GATE[1]:
                    r = cst.analyse(one)
                    ck, crms, cd, ch, cs0 = r['change']
                    if r['seen'] and ck == 'exit':
                        exit_checks.append((t, s_axle, cs0 - ahead))
            if name == 'E' and use_fixes:
                ahead = next_exit(path, s_axle)
                if ahead is not None:
                    r = cst.analyse(one)
                    ck, crms, cd, ch, cs0 = r['change']
                    if r['seen'] and ck == 'exit':
                        corr = float(np.clip(cs0 - ahead, -0.5, 0.5))
                        exit_fixes.append((t, corr))
                        s0_est += corr
        # parked at the start (on a straight, by procedure): gyro offset from a plain
        # straight-line fit, which doesn't depend on the (possibly wrong) position
        if dist < 0.02:
            for name, p in (('Y', yp), ('E', ep)):
                if len(p) < lc.MIN_POINTS // 2:
                    continue
                if len(p) > cot.MAX_YELLOW:
                    p = p[np.linspace(0, len(p) - 1, cot.MAX_YELLOW).astype(int)]
                one = {'yellow': p if name == 'Y' else p[:0], 'edge': p if name == 'E' else p[:0]}
                _, hs, rs = fit_dh(one, STRAIGHT)
                if rs < HEAD_RMS:
                    off = th_path + np.radians(hs) - gyro
                    g_n += 1
                    g_off = off if g_off is None else g_off + (off - g_off) / min(g_n, 10)
        src = 'DR'
        pick = None
        if 'Y' in res and 'E' in res:
            if abs(res['Y'][0] - res['E'][0]) < MAX_DISAGREE:
                pick = ('YE', (res['Y'][0] + res['E'][0]) / 2, (res['Y'][1] + res['E'][1]) / 2,
                        min(res['Y'][2], res['E'][2]))
        elif 'E' in res:
            pick = ('E', *res['E'])
        elif 'Y' in res:
            pick = ('Y', *res['Y'])
        if pick:
            src, d = pick[0], pick[1]
            # heading: camera only when its fit is very clean and plausible, else keep the gyro value
            if pick[3] < HEAD_RMS and abs(pick[2] - h_exp) < MAX_HEAD:
                h = pick[2]
                # on a straight (no turn between the rear axle and ~10 cm ahead) the
                # body heading is the lane direction + h: learn the gyro offset
                if np.ptp(np.interp(np.linspace(s_axle - WHEELBASE - 0.05, s_axle + 0.1, 10),
                                    path['s'], path['th'])) < np.radians(1):
                    off = th_path + np.radians(h) - gyro
                    g_n += 1
                    g_off = off if g_off is None else g_off + (off - g_off) / min(g_n, 10)
        rows.append(dict(t=t, dist=dist, s=s_axle, s_err=s0_est - s0_true, src=src, d=d, h=h, h_exp=h_exp,
                         dY=res.get('Y', (np.nan,) * 3)[0], dE=res.get('E', (np.nan,) * 3)[0]))
        prev = dict(dist=dist, gyro=gyro, th_path=th_path, h_exp=h_exp)

    tag = (os.path.basename(folder) + f'_fx{lc.FX:.0f}' + (f'_shift{s_start_err:+.2f}' if s_start_err else '')
           + f'_{mode}' + (f'_enc{enc_scale:.3f}' if enc_scale != 1.0 else ''))
    out = os.path.join(os.path.dirname(folder), f'c2_{tag}')
    with open(out + '.csv', 'w') as f:
        f.write('t,dist,s,s_err,src,d,h,h_exp,dY,dE\n')
        for r in rows:
            f.write(f"{r['t']:.3f},{r['dist']:.3f},{r['s']:.3f},{r['s_err']:.3f},{r['src']},{r['d']:.2f},{r['h']:.2f},{r['h_exp']:.2f},{r['dY']:.2f},{r['dE']:.2f}\n")
    plot(out + '.png', rows, path)
    if use_fixes:
        print('curve-end corrections (ahead_map - s0_cam, m):', ' '.join(f'{x:+.3f}' for _, x in exit_fixes))
    else:
        print('camera curve-end checks (s, camera - map, m; not applied):',
              ' '.join(f'{sa:.2f}:{x:+.3f}' for _, sa, x in exit_checks) or 'none')
    print('position error s_est - s_true (m) at s = ' +
          ', '.join(f"{r['s']:.2f}:{r['s_err']:+.3f}" for r in rows[::10]) + f", end:{rows[-1]['s_err']:+.3f}")
    mv = [r for r in rows if 0.02 < r['dist'] < odo['dist_m'][-1] - 0.02]
    d_all = np.array([r['d'] for r in mv])
    print(f"moving frames {len(mv)}: d mean {d_all.mean():+.1f} cm, sd {d_all.std():.1f}, "
          f"max |d| {np.abs(d_all).max():.1f} cm; sources " +
          ' '.join(f"{k}={sum(r['src'] == k for r in mv)}" for k in ('YE', 'E', 'Y', 'DR')))
    return rows


def plot(fn, rows, path):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    s = np.array([r['s'] for r in rows])
    fig, ax = plt.subplots(3, 1, figsize=(11, 8), sharex=True)
    col = {'YE': 'green', 'E': 'blue', 'Y': 'orange', 'DR': 'red'}
    for k, c in col.items():
        m = np.array([r['src'] == k for r in rows])
        ax[0].plot(s[m], np.array([r['d'] for r in rows])[m], '.', color=c, label=k)
        ax[1].plot(s[m], np.array([r['h'] for r in rows])[m], '.', color=c)
    ax[0].plot(s, [r['dY'] for r in rows], 'x', color='orange', ms=3, alpha=0.4, label='yellow fit (all)')
    ax[0].plot(s, [r['dE'] for r in rows], '+', color='blue', ms=3, alpha=0.4, label='edge fit (all)')
    ax[0].axhline(0, color='k', lw=0.5)
    ax[0].set_ylim(-15, 15)
    ax[0].set_ylabel('offset d (cm, + left)')
    ax[0].legend(fontsize=7, ncol=6)
    ax[1].plot(s, [r['h_exp'] for r in rows], 'k--', lw=0.8, label='expected (centred car)')
    ax[1].legend(fontsize=7)
    ax[1].set_ylim(-25, 20)
    ax[1].set_ylabel('heading to lane (deg)')
    ax[1].axhline(0, color='k', lw=0.5)
    ax[2].plot(path['s'], path['k'], 'k-')
    ax[2].set_ylabel('map curvature (1/m)')
    ax[2].set_xlabel('front axle position along the path s (m, node 0 = 0)')
    ax[2].set_xlim(s.min() - 0.1, s.max() + 0.1)
    fig.tight_layout()
    fig.savefig(fn, dpi=90)
    plt.close(fig)


if __name__ == '__main__':
    main()
