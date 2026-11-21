"""
fusion_drive_checks.py  (PC only)

Per-drive checks for the fusion filter (camera report §48-§49):
  1. GPS minus the camera-based estimate at the GPS point, along and across the
     route, in 0.5 m bins (only where the camera-based offset sd < 1.5 cm);
  2. node 2 re-acquisition (first camera fit after the 0 -> 2 gap): prediction per
     GPS mode, the C3 replay's prediction, s at the curve entry vs the car's, and
     where the offset sd exceeds 3 cm;
  3. sensitivity of the node 2 prediction to the along-route position at the entry;
  4. camera heading while parked.

    python tools/replay/fusion_drive_checks.py <folder with odometry_drive.csv, vehicle_control.log, lane_camera.csv> <c3_replay_<drive>.csv>

The c3_replay csv comes from c3_replay.py run with the drive's lane_camera.csv (4th argument).
"""
import os
import sys
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
import common  # noqa: E402  qcar/, tools/ folders and ../libraries/python on sys.path
import fusion_replay as fr
import lane_fusion as lf
from common import load_vc


def node2(rows, L):
    """Index of the first used camera fit at node 2 (after the 0 -> 2 gap)."""
    return next(i for i, r in enumerate(rows) if r['used'] == 'ok' and 1.9 < np.mod(r['s'], L) < 2.5)


def entry_offset(rows, odo, L):
    """Filter s minus the car's logged s at the 0 -> 2 curve entry (m)."""
    k = next(i for i, r in enumerate(rows) if 0.30 < np.mod(r['s'], L) < 0.5)
    car_s = odo['cs_s'][np.argmin(abs(odo['t_epoch'] - rows[k]['t']))]
    return np.mod(rows[k]['s'] - car_s + L / 2, L) - L / 2


def main():
    vc = load_vc()
    from hal.products.mats import SDCSRoadMap
    route = vc['RoutePath'](vc['remove_backward_points'](
        SDCSRoadMap(leftHandTraffic=False).generate_path(vc['nodeSequence'])))
    L = route.L
    D, c3csv = sys.argv[1], sys.argv[2]
    odo, gps, cam = fr.load(os.path.join(D, 'odometry_drive.csv'), os.path.join(D, 'vehicle_control.log'),
                            os.path.join(D, 'lane_camera.csv'))
    old = np.genfromtxt(c3csv, delimiter=',', names=True, dtype=None, encoding=None)

    # 1. GPS sideways / along vs the camera-based estimate (mode none: camera + odometry only)
    rows, _, _, _ = fr.run(vc, route, odo, gps, cam, 'none')
    rt = np.array([r['t'] for r in rows])
    f = lf.LaneFusion(route, vc['cs_wheelbase'], vc['gps_front_ahead'], 0, 0)
    print('1. GPS minus camera-based estimate at the GPS point (cm)')
    bins = {}
    for g in gps:
        j = np.searchsorted(rt, g[0])
        if j == 0 or j >= len(rt) or rt[j] - g[0] > 0.2:
            continue
        r = rows[j - 1]
        if r['sd_d'] > 0.015:
            continue
        f.x = np.array([r['s'], r['d'], r['h'] + route.heading(r['s']), 0, 0])
        pp = f._gps_point(f.x, bias=False)
        sp, latp = route.project(pp[0], pp[1], r['s'] - 0.11, 0.4)
        sg, latg = route.project(g[1], g[2], r['s'] - 0.11, 0.6)
        if abs(sg - sp) > 0.3 or abs(latg - latp) > 0.3:
            continue
        k = round(np.mod(r['s'], L) * 2) / 2
        bins.setdefault(k, []).append((latg - latp, sg - sp))
    for k in sorted(bins, key=lambda v: (v < 10, v)):
        a = np.array(bins[k])
        print(f'   s ~ {k:5.1f}: across {100 * a[:, 0].mean():+5.1f} (sd {100 * a[:, 0].std():.1f}), '
              f'along {100 * a[:, 1].mean():+5.1f}, n {len(a)}')

    # 2. node 2 re-acquisition per mode
    print('2. node 2 re-acquisition')
    for mode in ('full', 'along', 'none'):
        rows, _, _, _ = fr.run(vc, route, odo, gps, cam, mode)
        j = node2(rows, L)
        ds = entry_offset(rows, odo, L)
        sdd = np.array([r['sd_d'] for r in rows])
        sm = np.mod([r['s'] for r in rows], L)
        e = np.flatnonzero(np.diff(np.r_[0, (sdd > 0.03).astype(int), 0]))
        segs = ', '.join(f'{sm[a]:.2f}-{sm[b - 1]:.2f} (max {100 * sdd[a:b].max():.1f})' for a, b in zip(e[::2], e[1::2]))
        print(f'   [{mode}] prediction {100 * rows[j - 1]["d"]:+.1f} +- {100 * rows[j - 1]["sd_d"]:.1f}, '
              f'camera {100 * rows[j]["cam_d"]:+.1f} (s mod L {sm[j]:.2f}); s at curve entry vs car {100 * ds:+.1f} cm; '
              f'sd_d > 3 cm: {segs}')
    j = next(i for i in range(len(old)) if 1.9 < np.mod(old['s'][i], L) < 2.5 and old['src'][i] in ('YE', 'E', 'Y'))
    print(f'   [C3] prediction {old["d_cm"][j - 1]:+.1f} (replay row before the first fit)')

    # 3. sensitivity to the along-route position at the curve entry (mode none, start s shifted)
    print('3. sensitivity: node 2 prediction vs s at the curve entry (mode none)')
    orig = lf.LaneFusion.__init__
    for sh in (-0.08, -0.06, -0.04, -0.02, 0.0, 0.02):
        def init(self, r_, wb, ga, s0, psi0, d0=0.0, _sh=sh, **kw):
            orig(self, r_, wb, ga, s0 + _sh, psi0, d0, **kw)
        lf.LaneFusion.__init__ = init
        rows, _, _, _ = fr.run(vc, route, odo, gps, cam, 'none')
        j = node2(rows, L)
        print(f'   start {100 * sh:+3.0f} cm: s at entry vs car {100 * entry_offset(rows, odo, L):+5.1f} cm -> '
              f'node 2 {100 * rows[j - 1]["d"]:+5.1f} (camera {100 * rows[j]["cam_d"]:+.1f})')
    lf.LaneFusion.__init__ = orig

    # 4. parked camera heading
    t_move = odo['t_epoch'][np.nonzero(odo['dist_m'] > 0.002)[0][0]]
    m = (cam['t_epoch'] < t_move) & (cam['t_epoch'] > odo['t_epoch'][0]) & np.isin(cam['mf_src'], ['YE', 'E', 'Y'])
    print(f'4. parked camera heading {np.nanmean(cam["mf_h_deg"][m]):+.2f} deg (n {m.sum()})')


if __name__ == '__main__':
    main()
