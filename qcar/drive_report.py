"""
drive_report.py  (PC only)

Camera report §66: one command per drive folder for the checks done by hand after
every drive (steer_check.py, the stretch tables, re-acquisitions, along-route
corrections, GPS status). Route-agnostic: the route is split into straights and arcs
from the map's own curvature and each stretch is labelled by the nodes around it.

    python qcar/drive_report.py <folder> [--frames]
<folder> holds odometry_drive.csv, vehicle_control.log, lane_camera.csv (and the raw
frames folder). --frames also runs the camera along-route measurement on the saved
frames (shift_meas.py -> <folder>/shift_meas.csv) and lists where it finds a shift.

Sections:
  1. summary: distance, time, speeds, stop reason, estimator, corrections, GPS
  2. per stretch: camera fits used / dead reckoning, camera-measured offset (mean,
     min, max), commanded / achieved / needed steering, time at the steering limit
  3. steering calibration: straight-line command and achieved angle, achieved/commanded
     in curves, longest time at the limit
  4. camera re-acquisitions after > 0.5 m without a fit: estimate before vs camera
  5. along-route corrections, ignored camera fits, GPS acceptance by stretch
"""
import os
import re
import sys
import subprocess
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, '..', '..', 'libraries', 'python'))
from c3_replay import load_vc

FIT_SRC = ('E', 'Y', 'YE')
GAP = 0.5            # m without a fit before a re-acquisition counts
K_STRAIGHT = 0.3     # 1/m: below this |curvature| a stretch is a straight
MIN_LEN = 0.15       # m: shorter stretches are merged into the neighbour


def route_and_nodes():
    vc = load_vc()
    from hal.products.mats import SDCSRoadMap
    rm = SDCSRoadMap(leftHandTraffic=False)
    route = vc['RoutePath'](vc['remove_backward_points'](rm.generate_path(vc['nodeSequence'])))
    nodes = []
    for n in vc['nodeSequence'][:-1]:
        p = rm.nodes[n].pose[:2].ravel()
        nodes.append((n, float(route.s[np.argmin(np.hypot(route.x - p[0], route.y - p[1]))])))
    return vc, route, nodes


def stretches(route, nodes):
    """Straights and constant-curvature arcs of the route: (s0, s1, kappa, label)."""
    k = route.k.copy()
    s = route.s
    kind = np.where(np.abs(k) < K_STRAIGHT, 0.0, np.round(k, 1))
    out = []
    i = 0
    while i < len(s):
        j = i
        while j + 1 < len(s) and kind[j + 1] == kind[i]:
            j += 1
        out.append([s[i], s[j] + (s[1] - s[0]), float(np.median(k[i:j + 1])) if kind[i] else 0.0])
        i = j + 1
    merged = []
    for st in out:                       # merge tiny pieces (curvature transitions)
        if merged and st[1] - st[0] < MIN_LEN:
            merged[-1][1] = st[1]
        else:
            merged.append(st)
    ns = sorted(nodes, key=lambda x: x[1])
    res = []
    for s0, s1, kk in merged:
        inside = [n for n, sn in ns if s0 - 0.01 <= sn <= s1 - 0.05]   # nodes within the stretch
        if len(inside) >= 2:
            where = f'{inside[0]}->{inside[-1]}'
        elif len(inside) == 1:
            where = f'at {inside[0]}'
        else:                                                      # between the node before and after
            before = [n for n, sn in ns if sn <= s1 - 0.05]
            after = [n for n, sn in ns if sn >= s0 + 0.05]
            where = f'{before[-1] if before else ns[-1][0]}->{after[0] if after else ns[0][0]}'
        kind_txt = 'straight' if kk == 0 else f'arc k {kk:+.2f}'
        res.append((s0, s1, kk, f'{where} {kind_txt}'))
    return res


def needed_steer(kk, L):
    """Steering for the front axle to follow an arc of curvature kk (front-axle radius)."""
    if abs(kk) < 1e-3:
        return 0.0
    R = 1 / abs(kk)
    return np.sign(kk) * np.degrees(np.arctan(L / np.sqrt(max(R * R - L * L, 1e-6))))


def main():
    folder = sys.argv[1].rstrip('/\\')
    want_frames = '--frames' in sys.argv
    vc, route, nodes = route_and_nodes()
    RL, L = route.L, vc['cs_wheelbase']
    o = np.genfromtxt(os.path.join(folder, 'odometry_drive.csv'), delimiter=',', names=True, dtype=None, encoding=None)
    log = open(os.path.join(folder, 'vehicle_control.log'), encoding='utf-8', errors='replace').read().splitlines()
    t = o['t_epoch']
    dt = np.median(np.diff(t))
    n = max(1, int(round(0.2 / dt)))
    ker = np.ones(n) / n
    gz = np.convolve(o['gyro_z'], ker, 'same')
    v = np.convolve(o['tach_mps'], ker, 'same')
    ach = np.degrees(np.arctan2(L * gz, np.maximum(v, 1e-3)))
    lag = max(1, int(round(0.1 / dt)))
    cmd = np.degrees(np.r_[np.full(lag, np.nan), o['delta'][:-lag]])   # command 0.1 s earlier (servo)
    S = o['cs_s']
    s = np.mod(S, RL)
    d = o['cs_d_cm']
    src = o['cs_src'].astype(str)
    moving = v > 0.05
    st_list = stretches(route, nodes)

    # ---- 1. summary
    stop = next((l for l in log if l.startswith('STOPPED')), None)
    est = next((l for l in log if 'estimator =' in l), None)
    corr = [l for l in log if 'along-route correction' in l or 'along-route measurement' in l]
    ign = [l for l in log if 'camera offset' in l and 'ignored' in l]
    relock = [l for l in log if 're-lock' in l]
    gps_ok = [l for l in log if l.startswith('GPS:')]
    gps_rej = [l for l in log if l.startswith('GPS rejected')]
    tm = t[moving]
    print(f'== {folder}')
    print(f'1. distance {S[-1] - S[0]:.2f} m (s {S[0]:.2f} -> {S[-1]:.2f}, route length {RL:.2f}), '
          f'{(tm[-1] - tm[0]) if len(tm) else 0:.1f} s moving, mean speed {v[moving].mean() if moving.any() else 0:.3f} m/s')
    print(f'   stop: {stop if stop else "no STOPPED line (hand stop or end of log)"}')
    print(f'   estimator: {est.split("estimator = ")[1] if est else "C3 (no estimator line)"}; '
          f'along-route corrections {len(corr)}; camera fits ignored by the gate {len(ign)}, re-locks {len(relock)}; '
          f'GPS lines accepted {len(gps_ok)}, rejected by the jump check {len(gps_rej)}')

    # ---- 2. per stretch (first pass of each stretch only, after the start)
    print('\n2. per stretch (route s; first pass): camera fits used / rows, offset where the camera measured it (cm), '
          'steering cmd / achieved (mean over the stretch, entry included) / needed in the arc (deg), % at the limit')
    print(f'   {"stretch":24s} {"s":>11s} {"fits/rows":>10s} {"off mean":>8s} {"min":>6s} {"max":>6s} '
          f'{"cmd":>6s} {"ach":>6s} {"need":>6s} {"lim%":>5s}')
    lap_end = S[0] + RL
    for s0, s1, kk, lab in st_list:
        # rows of this stretch in the drive, unwrapped: the occurrence after the start
        k0 = np.ceil((S[0] - s1) / RL)
        rows = np.zeros(len(S), bool)
        for kk_lap in (k0, k0 + 1):
            rows |= (S >= s0 + kk_lap * RL) & (S < s1 + kk_lap * RL) & (S < lap_end + 0.01)
        rows &= moving
        if not rows.any():
            continue
        fit = rows & np.isin(src, FIT_SRC)
        need = needed_steer(kk, L)
        mid = rows & (np.abs(route.curvature(s) - kk) < 0.05) if kk else rows   # arc middle only for the steering
        if not mid.any():
            mid = rows
        dd = d[fit]
        lim = np.mean(np.abs(cmd[rows][~np.isnan(cmd[rows])]) > 29.5) * 100
        print(f'   {lab:24s} {s0:5.2f}-{s1:5.2f} {fit.sum():4d}/{rows.sum():<5d} '
              f'{(dd.mean() if len(dd) else np.nan):+8.1f} {(dd.min() if len(dd) else np.nan):+6.1f} '
              f'{(dd.max() if len(dd) else np.nan):+6.1f} {np.nanmean(cmd[mid]):+6.1f} {np.nanmean(ach[mid]):+6.1f} '
              f'{need:+6.1f} {lim:5.0f}')

    # ---- 3. steering calibration
    win = np.linspace(-0.3, 0.6, 19)          # straight from 0.3 m behind to 0.6 m ahead (no transitions)
    straight = moving & ~np.isnan(cmd) & np.array([np.max(np.abs(route.curvature(si + win))) < 0.05 for si in s])
    print('\n3. steering calibration')
    if straight.any():
        print(f'   straights: command {np.nanmean(cmd[straight]):+.2f} deg, achieved {np.nanmean(ach[straight]):+.2f} deg '
              f'(both ~0 when steering_trim is right; achieved - command {np.nanmean(ach[straight] - cmd[straight]):+.2f})')
    for lo, hi in ((10, 15), (15, 20), (20, 25), (25, 31)):
        m = moving & (np.abs(cmd) >= lo) & (np.abs(cmd) < hi)
        if m.sum() > 5:
            print(f'   |command| {lo}-{hi} deg: achieved/commanded {np.mean(np.abs(ach[m])) / np.mean(np.abs(cmd[m])):.2f} '
                  f'({np.mean(np.abs(ach[m])):.1f} for {np.mean(np.abs(cmd[m])):.1f}, n {m.sum()})')
    lim = np.abs(np.degrees(o['delta'])) > 29.5
    best = cur = 0
    for x in lim:
        cur = cur + 1 if x else 0
        best = max(best, cur)
    print(f'   longest time at the steering limit {best * dt:.2f} s')

    # ---- 4. re-acquisitions (the car's own estimate before the first fit after a gap)
    print('\n4. camera re-acquisitions after > 0.5 m without a used fit: estimate before / camera (cm)')
    last_fit_s = None
    items = []
    for i in range(1, len(S)):
        if src[i] in FIT_SRC:
            if last_fit_s is not None and S[i] - last_fit_s > GAP and src[i - 1] not in FIT_SRC:
                items.append((s[i], d[i - 1], d[i], S[i] - last_fit_s))
            last_fit_s = S[i]
    if items:
        e = np.array([a - b for _, a, b, _ in items])
        print(f'   {len(items)} re-acquisitions, |error| mean {np.mean(np.abs(e)):.1f} cm, max {np.max(np.abs(e)):.1f}')
        for si, a, b, g in items:
            print(f'   s {si:5.2f} after {g:.2f} m: {a:+5.1f} / {b:+5.1f}  (error {a - b:+.1f})')
    else:
        print('   none')
    for l in ign[:10]:
        m = re.search(r'camera offset ([-+\d.]+) cm ignored \(predicted ([-+\d.]+).*s = ([\d.]+) m', l)
        if m:
            print(f'   ignored by the gate at s {np.mod(float(m.group(3)), RL):.2f}: camera {m.group(1)} cm, estimate {m.group(2)} cm')

    # ---- 5. corrections and GPS by stretch
    print('\n5. along-route corrections')
    for l in corr:
        print('   ' + re.sub(r'\s+t_epoch=.*', '', l.replace('CAMSTEER: ', '')))
    if not corr:
        print('   none')
    gt = []
    for l in gps_ok + gps_rej:
        m = re.search(r't_epoch=([\d.]+)', l)
        if m:
            gt.append((float(m.group(1)), l.startswith('GPS:')))
    if gt:
        gt = np.array(gt)
        gs = np.mod(np.interp(gt[:, 0], t, S), RL)
        print('   GPS accepted / rejected by stretch (the car\'s jump check):')
        for s0, s1, kk, lab in st_list:
            m = (gs >= s0) & (gs < s1)
            if m.any():
                print(f'     {lab:24s} {int(gt[m, 1].sum()):3d} / {int((1 - gt[m, 1]).sum()):3d}')

    if want_frames:
        frames = [f for f in os.listdir(folder) if os.path.isdir(os.path.join(folder, f)) and re.match(r'\d{8}_\d{6}$', f)]
        if frames:
            subprocess.run([sys.executable, os.path.join(HERE, 'shift_meas.py'), os.path.join(folder, frames[0]),
                            os.path.join(folder, 'odometry_drive.csv')], check=True)
            sm = np.genfromtxt(os.path.join(folder, 'shift_meas.csv'), delimiter=',', names=True)
            sm = np.atleast_1d(sm)
            ok = sm[sm['ok'] == 1] if len(sm) else sm
            print('   camera along-route shift on the frames (usable): ' +
                  (', '.join(f's {np.mod(r["s_logged"], RL):.2f}: {100 * r["sh_m"]:+.0f} cm' for r in ok) or 'none'))


if __name__ == '__main__':
    main()
