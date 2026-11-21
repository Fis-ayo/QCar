"""
replay_check.py  (PC only)

Camera report §66: the fixed check to run before every estimator change. Replays all
listed drives through the fusion filter (lane_fusion.LaneFusion via fusion_replay.run,
GPS 'full', camera fits, camera along-route measurements from shift_meas.csv, made
from the frames if missing) and prints per drive and in total:
  - camera fits used / rejected (a rejected real fit is how drive F failed, §63)
  - camera re-acquisitions after > 0.5 m without a fit: mean / max |error|
  - consistency: NIS per dimension of the camera updates, mean and 99th percentile. The
    mean is << 1 (13 fits/s that agree: under-confident on average); the overconfidence
    that broke drive F is local, so the flag is the 99th percentile > 9 (the 1-D rejection
    threshold)
  - along-route measurements used / rejected
  - checks against ground truth where we have it: the ruler (front -11 / rear -3 cm,
    §55), drive D's S-bend (camera +19 cm at s 10.68, §58), drive F's roundabout (§63),
    car B drive B's blind 12 -> 0 straight (camera -12 cm at its end, §70)
and a PASS / CHECK verdict per drive (rejections, re-acquisition > 5 cm, NIS p99 > 9).

    python tools/replay/replay_check.py [--dr axle|old] [--enc front|rear|mean] [--set name=value ...] [drive ...]
--set overrides lane_fusion.DEFAULTS for this run, e.g. --set q_s=0.1 --set cam_sd_h_deg=6
(cam_sd_h_deg / cam_sd0_scale are shortcuts). Default: dr axle, enc front, all drives.
"""
import os
import sys
import subprocess
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
import common  # noqa: E402  qcar/, tools/ folders and ../libraries/python on sys.path
import fusion_replay as fr
import lane_fusion as lf
from common import load_vc, pop_opt

ROOT = common.LOGS
A, B = os.path.join(ROOT, 'car A'), os.path.join(ROOT, 'car B')
# name: (folder with odometry_drive.csv / vehicle_control.log / lane_camera.csv, raw frames, shift_meas file)
DRIVES = {
    'A40':  (A, os.path.join(A, 'raw_frames', '20261118_234715'), os.path.join(A, 'raw_frames', 'shift_meas_20261118_234715.csv')),
    'A46':  (os.path.join(A, 'Fresh_Log'), os.path.join(A, 'Fresh_Log', '20261119_234304'), None),
    'A_A':  (os.path.join(A, '20261120_driveA'), os.path.join(A, '20261120_driveA', '20261120_190909'), None),
    'A_B':  (os.path.join(A, '20261120_driveB'), os.path.join(A, '20261120_driveB', '20261120_193052'), None),
    'A_C':  (os.path.join(A, '20261120_driveC'), os.path.join(A, '20261120_driveC', '20261120_194525'), None),
    'ruler': (os.path.join(A, '20261120_rulercheck'), os.path.join(A, '20261120_rulercheck', '20261120_201634'), None),
    'A_D':  (os.path.join(A, '20261120_driveD'), os.path.join(A, '20261120_driveD', '20261120_212931'), None),
    'A_E':  (os.path.join(A, '20261120_driveE'), os.path.join(A, '20261120_driveE', '20261120_215026'), None),
    'A_F':  (os.path.join(A, '20261120_driveF'), os.path.join(A, '20261120_driveF', '20261120_223328'), None),
    'B_A':  (os.path.join(B, '20261120_driveA'), os.path.join(B, '20261120_driveA', '20261120_220828'), None),
    'A_G':  (os.path.join(A, '20261120_driveG'), os.path.join(A, '20261120_driveG', '20261120_231750'), None),
    'B_B':  (os.path.join(B, '20261120_driveB'), os.path.join(B, '20261120_driveB', '20261120_232706'), None),
    'A_H':  (os.path.join(A, '20261120_driveH'), os.path.join(A, '20261120_driveH', '20261120_234802'), None),
}


def rear_offset(route, wb, s, d, h):
    th = float(route.heading(s))
    F = route.xy(s) + d * np.array([-np.sin(th), np.cos(th)])
    psi = th + h
    R = F - wb * np.array([np.cos(psi), np.sin(psi)])
    sg = s + np.linspace(-0.6, 0.1, 701)
    XY = route.xy(sg)
    j = np.argmin(((XY - R) ** 2).sum(1))
    tt = route.heading(sg[j])
    v = R - XY[j]
    return -v[0] * np.sin(tt) + v[1] * np.cos(tt)


def main():
    dr = pop_opt('--dr', 'axle', ('axle', 'old'))
    enc = pop_opt('--enc', 'front', lf.ENC_AXLES)
    over = {}
    while '--set' in sys.argv:
        k = sys.argv.index('--set')
        key, val = sys.argv[k + 1].split('=')
        del sys.argv[k:k + 2]
        if key == 'cam_sd_h_deg':
            x = float(val)
            over['cam_sd_h'] = {'YE': np.radians(x * 2 / 3), 'E': np.radians(x), 'Y': np.radians(x)}
        elif key == 'shift_sd':
            fr.shift_sd_clean, fr.shift_sd = float(val), 1.75 * float(val)
        elif key == 'cam_sd0_scale':
            over['cam_sd0'] = {kk: v * float(val) for kk, v in lf.DEFAULTS['cam_sd0'].items()}
        else:
            over[key] = float(val)
    names = sys.argv[1:] or list(DRIVES)
    vc = load_vc()
    from hal.products.mats import SDCSRoadMap
    route = vc['RoutePath'](vc['remove_backward_points'](
        SDCSRoadMap(leftHandTraffic=False).generate_path(vc['nodeSequence'])))
    L, WB = route.L, vc['cs_wheelbase']
    base = dict(lf.DEFAULTS)
    print(f'replay_check: dr {dr}, enc {enc}, overrides {over or "none"}')
    print('cam used/rej+impossible: "impossible" = rejected fits beyond the lane edge (wrong fits, correctly rejected)')
    print(f'{"drive":6s} {"cam used/rej":>12s} {"reacq n":>7s} {"|err| mean":>10s} {"max":>5s} {"NIS/dim":>7s} '
          f'{"along used/rej":>14s}  truth check / verdict   (NIS/dim: mean/p99)')
    tot = []
    for name in names:
        folder, frames, shf = DRIVES[name]
        shf = shf or os.path.join(folder, 'shift_meas.csv')
        if not os.path.exists(shf):
            subprocess.run([sys.executable, common.tool('checks', 'shift_meas.py'), frames,
                            os.path.join(folder, 'odometry_drive.csv')], check=True, capture_output=True)
        odo, gps, cam = fr.load(os.path.join(folder, 'odometry_drive.csv'), os.path.join(folder, 'vehicle_control.log'),
                                os.path.join(folder, 'lane_camera.csv'))
        lf.DEFAULTS.clear()
        lf.DEFAULTS.update(base)
        lf.DEFAULTS.update(over)
        sh = np.atleast_1d(np.genfromtxt(shf, delimiter=',', names=True))
        sh = np.array([(r['t_epoch'], r['s_true'], r['rms'], r['ok']) for r in sh],
                      dtype=[('t_epoch', float), ('s_true', float), ('rms', float), ('ok', int)])
        if 'sh_ok' in cam.dtype.names:            # live measurements the car received (frames are only 5/s)
            lv = cam[(cam['sh_ok'] == 1) & np.isfinite(cam['mf_s'])]
            if len(lv):
                add = np.array([(r['t_epoch'], r['mf_s'] + r['sh_m'], r['sh_rms'], 1) for r in lv], dtype=sh.dtype)
                sh = np.sort(np.concatenate([sh, add]), order='t_epoch')
        fr.OPT.update(dr=dr, enc=enc, shift=sh)
        rows, _, n, _ = fr.run(vc, route, odo, gps, cam, 'full')
        # a rejected fit beyond the lane edge is a wrong fit correctly rejected (island lines, §46/§70)
        impossible = lambda r: r['used'] == 'rej' and abs(r['cam_d']) > lf.LANE_HALF
        n_rej = sum(1 for r in rows if r['used'] == 'rej' and not impossible(r))
        n_imp = sum(1 for r in rows if impossible(r))
        ra = [x for x in fr.reacq(rows, L) if not (x[3] == 'rej' and abs(x[2]) > lf.LANE_HALF)]
        e = np.array([p - c for _, p, c, _ in ra]) if ra else np.array([np.nan])
        nis = np.array([r['nis'] / r['ndim'] for r in rows if r['ndim'] and np.isfinite(r['nis'])])
        truth = ''
        if name == 'ruler':
            r = rows[-1]
            truth = f'ruler: front {100 * r["d"]:+.1f} rear {100 * rear_offset(route, WB, r["s"], r["d"], r["h"]):+.1f} (true -11/-3)'
        elif name == 'A_D':
            p = [r for r in rows if 10.3 <= np.mod(r['s'], L) < 10.68 and (r['s'] - rows[0]['s']) > 5]
            truth = f'S-bend: estimate {100 * np.mean([r["d"] for r in p]):+.1f} before the camera saw +19' if p else ''
        elif name == 'B_B':
            p = [r for r in rows if 12.6 <= np.mod(r['s'], L) < 13.4 and (r['s'] - rows[0]['s']) > 8]
            truth = (f'12->0 straight: estimate {100 * p[0]["d"]:+.1f} -> {100 * p[-1]["d"]:+.1f} before the camera saw -12'
                     if p else '')
        elif name in ('A_H', 'A_F'):
            w = [(r['d_pred'] - r['cam_d']) for r in rows if r['used'] in ('ok', 'rej') and 8.0 <= np.mod(r['s'], L) < 8.3
                 and (r['s'] - rows[0]['s']) > 5]
            truth = f'roundabout 8.0-8.3: prediction - camera {100 * np.mean(w):+.1f} cm (n {len(w)})' if w else ''
        elif name == 'A_F_unused':
            rej = sum(1 for r in rows if r['used'] == 'rej' and 7.6 <= np.mod(r['s'], L) < 9.9)
            truth = f'roundabout rejections {rej} (car: 1)'
        bad = []
        if n_rej:
            bad.append('rejections')
        if np.nanmean(np.abs(e)) > 0.05:
            bad.append('re-acq > 5 cm')
        p99 = np.percentile(nis, 99) if len(nis) else np.nan
        if len(nis) and p99 > 9:
            bad.append('overconfident (NIS p99 > 9)')
        verdict = 'PASS' if not bad else 'CHECK: ' + ', '.join(bad)
        tot.append((n_rej, np.nanmean(np.abs(e)), p99, not bad))
        print(f'{name:6s} {n["cam"]:6d}/{n_rej:<2d}+{n_imp:<2d} {len(ra):7d} {100 * np.nanmean(np.abs(e)):10.1f} '
              f'{100 * np.nanmax(np.abs(e)):5.1f} {(np.mean(nis) if len(nis) else np.nan):5.2f}/{p99:5.1f} '
              f'{n["sh"]:8d}/{n["sh_rej"]:<5d}  {truth}{"  " if truth else ""}{verdict}', flush=True)
    lf.DEFAULTS.clear()
    lf.DEFAULTS.update(base)
    T = np.array(tot, float)
    print(f'TOTAL  rejections {int(T[:, 0].sum())}, re-acquisition |error| mean {100 * np.nanmean(T[:, 1]):.1f} cm, '
          f'NIS/dim p99 max {np.nanmax(T[:, 2]):.1f}, PASS {int(T[:, 3].sum())}/{len(T)}')


if __name__ == '__main__':
    main()
