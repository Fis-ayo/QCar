"""
fusion_replay.py  (PC only)

Replays a camera-steering drive through lane_fusion.LaneFusion (camera report
"Where to pick up" step 3), using the logged message stream: odometry_drive.csv
(encoder, gyro), the GPS lines of vehicle_control.log (accepted AND rejected by the
car: the filter makes its own decision) and the map fits in lane_camera.csv.
The steering isn't simulated: the car was steered by the C3 estimator, so this
checks the estimate only.

    python tools/replay/fusion_replay.py <odometry_drive.csv> <vehicle_control.log> <lane_camera.csv> [c3_replay_<drive>.csv]
        [--dr old|axle] [--enc front|rear|mean] [--shift <shift_meas.csv>]
--dr axle: the §53-§54 axle dead reckoning; --enc: which axle the encoder distance is;
--shift: camera along-route measurements from shift_meas.py (§61). Suffix on the outputs.
The optional 4th argument is the C3 estimator's replay of the same drive (c3_replay.py
with lane_camera.csv), drawn for comparison; it also names the output files.

What it prints / writes (next to the odometry file):
  1. full replay, GPS mode 'full': camera fits used/rejected, GPS used/rejected,
     where the offset uncertainty first exceeds sd_stop (the uncertainty stop that
     would replace cs_dr_max), and the same for GPS modes 'along' and 'none';
  2. held-out test: camera fits hidden over hold_len m windows on stretches where
     the camera is good; the filter's offset just before the first fit after the
     window is compared with that fit (error and error / predicted sd), for the
     GPS modes ('full' with bias states, 'white' without, 'along', 'none');
  fusion_replay_<drive>.csv / .png.
"""
import os
import re
import sys
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
import common  # noqa: E402  qcar/, tools/ folders and ../libraries/python on sys.path
from common import load_vc
from lane_fusion import LaneFusion, wrap, ENC_AXLES
from common import pop_opt

# §61 options: dead-reckoning model, which axle the encoder is, camera along-route measurements
OPT = dict(dr='old', enc='front', shift=None)
shift_sd_clean, shift_sd = 0.04, 0.07    # m: sd of a camera along-route measurement (fit rms < 0.06 / otherwise)

sd_stop = 0.03          # m, offset sd that would stop the car
hold_len = 1.0          # m, held-out window length
hold_step = 0.25        # m between held-out windows
hold_ranges = [(2.3, 4.4), (4.4, 7.4)]   # s mod L: node 2 -> node 4 (GPS good), 4 -> 14 (GPS lost)


def load(odo_file, log_file, cam_file):
    odo = np.genfromtxt(odo_file, delimiter=',', names=True, dtype=None, encoding=None)
    gps = []
    for line in open(log_file):
        m = re.match(r'GPS(?: rejected)?: \(([-\d.]+),([-\d.]+),([-\d]+)deg\).*t_epoch=([\d.]+)', line)
        if m:
            g = (float(m.group(4)), float(m.group(1)), float(m.group(2)), np.radians(float(m.group(3))))
            # the log repeats some readings many times within ~50 ms: keep one
            if gps and g[1:] == gps[-1][1:] and g[0] - gps[-1][0] < 0.1:
                continue
            gps.append(g)
    cam = np.genfromtxt(cam_file, delimiter=',', names=True, dtype=None, encoding=None)
    return odo, np.array(gps), cam


def run(vc, route, odo, gps, cam, gps_mode='full', hide=None, verbose=False):
    """hide: list of (t0, t1): camera fits (and camera along-route measurements) in
    these time windows are ignored. OPT: dr / enc / shift (array from shift_meas.csv)."""
    cps = odo['dist_m'][-1] / odo['encoder'][-1]
    moving = np.nonzero(odo['dist_m'] > 0.002)[0]
    t_move = odo['t_epoch'][moving[0]]
    bias = float(np.mean(odo['gyro_z_raw'][odo['t_epoch'] < t_move]))
    g0 = gps[gps[:, 0] >= odo['t_epoch'][0] - 5][0]
    sG0, _ = route.locate(g0[1], g0[2], g0[3], vc['cs_start_head'])
    f = LaneFusion(route, vc['cs_wheelbase'], vc['gps_front_ahead'], sG0 + vc['gps_front_ahead'], g0[3],
                   dr_model=OPT['dr'], enc_axle=OPT['enc'])
    sh = OPT['shift']
    si = 0 if sh is None else int(np.searchsorted(sh['t_epoch'], odo['t_epoch'][0]))
    cam_t = cam['t_epoch']
    gi = int(np.searchsorted(gps[:, 0], odo['t_epoch'][0]))
    ci = int(np.searchsorted(cam_t, odo['t_epoch'][0]))
    rows, gps_rows = [], []
    n = dict(cam=0, cam_rej=0, cam_hbad=0, gps=0, gps_rej=0, sh=0, sh_rej=0)
    sh_rows = []
    stop = None
    enc_prev = odo['encoder'][0]
    for i in range(len(odo)):
        t = odo['t_epoch'][i]
        if i:
            ds = vc['enc_scale'] * (odo['encoder'][i] - enc_prev) * cps
            f.predict(ds, (odo['gyro_z_raw'][i] - bias) * (t - odo['t_epoch'][i - 1]))
        enc_prev = odo['encoder'][i]
        while gi < len(gps) and gps[gi, 0] <= t:
            if gps_mode != 'none':
                ok, nis = f.gps(gps[gi, 1], gps[gi, 2], gps[gi, 3], gps_mode)
                n['gps' if ok else 'gps_rej'] += 1
                gps_rows.append((gps[gi, 0], f.x[0], ok, nis))
            gi += 1
        while sh is not None and si < len(sh) and sh['t_epoch'][si] <= t:
            r = sh[si]
            if r['ok'] == 1 and not (hide and any(a <= r['t_epoch'] <= b for a, b in hide)):
                s_before = f.x[0]
                ok, nis = f.camera_s(float(r['s_true']), shift_sd_clean if r['rms'] < 0.06 else shift_sd)
                n['sh' if ok else 'sh_rej'] += 1
                sh_rows.append((r['t_epoch'], s_before, float(r['s_true']), ok, nis))
            si += 1
        while ci < len(cam_t) and cam_t[ci] <= t:
            r = cam[ci]
            src = str(r['mf_src'])
            used = None
            d_pred = np.nan
            nis, ndim = np.nan, 0
            if src in ('YE', 'E', 'Y') and not (hide and any(a <= cam_t[ci] <= b for a, b in hide)):
                s = f.x[0]
                h_exp = f.h_exp()
                mh = np.radians(float(r['mf_h_deg']))
                if abs(wrap(mh - h_exp)) > vc['cs_max_head_fit']:
                    n['cam_hbad'] += 1          # the fit itself is implausible (§40), not an estimator test
                    used = 'hbad'
                else:
                    k_win = route.curvature(s + np.linspace(*vc['cs_head_window'], 37))
                    use_h = (np.ptp(k_win) <= vc['cs_head_kchange'] and float(r['mf_rms_cm']) < vc['cs_head_rms']
                             and abs(wrap(mh - h_exp)) < vc['cs_max_head'])
                    d_pred = f.x[1]
                    ok, nis = f.camera(src, float(r['mf_d_cm']) / 100, mh if use_h else None,
                                       float(r['mf_rms_cm']) / 100)
                    ndim = 2 if use_h else 1
                    n['cam' if ok else 'cam_rej'] += 1
                    used = 'ok' if ok else 'rej'
                    if verbose and not ok:
                        print(f'  camera rejected at s = {s:.2f}: {float(r["mf_d_cm"]):+.1f} cm ({src}), '
                              f'predicted {100 * d_pred:+.1f} +- {100 * f.sd()[1]:.1f}, nis {nis:.0f}')
            sd = f.sd()
            rows.append(dict(t=cam_t[ci], s=f.x[0], d=f.x[1], d_pred=d_pred if used in ('ok', 'rej') else np.nan,
                             nis=nis, ndim=ndim,
                             h=f.h(), h_exp=f.h_exp(),
                             sd_s=sd[0], sd_d=sd[1], sd_h=sd[2], src=src, used=used,
                             cam_d=float(r['mf_d_cm']) / 100 if src in ('YE', 'E', 'Y') else np.nan))
            ci += 1
        if stop is None and odo['dist_m'][i] > 0.002 and f.sd()[1] > sd_stop:
            stop = (t, f.x[0], f.sd()[0], f.sd()[2])
    run.sh_rows = sh_rows
    return rows, gps_rows, n, stop


def reacq(rows, L, gap=0.5):
    """Camera re-acquisitions after more than gap m without a used fit: (s, predicted d,
    camera d, used?). The prediction is the filter's offset just before that fit."""
    out, last_s = [], None
    for r in rows:
        if r['used'] in ('ok', 'rej'):
            if last_s is not None and r['s'] - last_s > gap:
                out.append((np.mod(r['s'], L), r['d_pred'], r['cam_d'], r['used']))
            if r['used'] == 'ok':
                last_s = r['s']
    return out


def held_out(vc, route, odo, gps, cam, ref_rows, mode):
    """Hide camera fits over windows; compare the prediction with the next fit."""
    L = route.L
    ts = np.array([r['t'] for r in ref_rows])
    ss = np.array([r['s'] for r in ref_rows])
    ok_fit = np.array([r['used'] == 'ok' for r in ref_rows])
    out = []
    for lo, hi in hold_ranges:
        for a in np.arange(lo, hi - hold_len + 1e-9, hold_step):
            inwin = (np.mod(ss, L) >= a) & (np.mod(ss, L) < a + hold_len)
            if not inwin.any():
                continue
            t0, t1 = ts[inwin][0], ts[inwin][-1]
            after = np.nonzero((ts > t1) & ok_fit)[0]
            if not len(after) or ss[after[0]] - ss[inwin][-1] > 0.3:
                continue
            rows, _, _, _ = run(vc, route, odo, gps, cam, mode, hide=[(t0, t1)])
            rt = np.array([r['t'] for r in rows])
            j = int(np.searchsorted(rt, ts[after[0]]))   # row of the reference fit
            pred, sdp = rows[j - 1]['d'], rows[j - 1]['sd_d']
            ref = rows[j]['cam_d']
            out.append((a, pred - ref, sdp))
    return np.array(out)


def main():
    OPT['dr'] = pop_opt('--dr', 'old', ('old', 'axle'))
    OPT['enc'] = pop_opt('--enc', 'front', ENC_AXLES)
    if '--shift' in sys.argv:
        k = sys.argv.index('--shift')
        OPT['shift'] = np.genfromtxt(sys.argv[k + 1], delimiter=',', names=True)
        del sys.argv[k:k + 2]
    odo_file, log_file, cam_file = sys.argv[1:4]
    vc = load_vc()
    from hal.products.mats import SDCSRoadMap
    rm = SDCSRoadMap(leftHandTraffic=False)
    route = vc['RoutePath'](vc['remove_backward_points'](rm.generate_path(vc['nodeSequence'])))
    odo, gps, cam = load(odo_file, log_file, cam_file)
    L = route.L
    print(f'{len(odo)} odometry rows, {len(gps)} GPS readings (duplicates dropped), {len(cam)} camera rows')

    results = {}
    for mode in ('full', 'white', 'along', 'none'):
        rows, gps_rows, n, stop = run(vc, route, odo, gps, cam, mode, verbose=(mode == 'full'))
        results[mode] = (rows, gps_rows)
        D = np.array([r['d'] for r in rows])
        print(f'\n[{mode}] camera fits used {n["cam"]}, rejected {n["cam_rej"]}, implausible heading {n["cam_hbad"]}; '
              f'GPS used {n["gps"]}, rejected {n["gps_rej"]}')
        print(f'  offset mean {100 * D.mean():+.1f} cm, sd {100 * D.std():.1f}, max |d| {100 * np.abs(D).max():.1f}; '
              f'end s = {rows[-1]["s"]:.2f} (mod L {np.mod(rows[-1]["s"], L):.2f})')
        if stop:
            print(f'  uncertainty stop (sd_d > {100 * sd_stop:.0f} cm) at s = {stop[1]:.2f} (mod L {np.mod(stop[1], L):.2f}), '
                  f'sd_s {100 * stop[2]:.0f} cm, sd_psi {np.degrees(stop[3]):.1f} deg')
        else:
            print(f'  no uncertainty stop (max sd_d {100 * max(r["sd_d"] for r in rows):.1f} cm)')
        if mode == 'full':
            if OPT['shift'] is not None:
                print(f'  camera along-route measurements used {n["sh"]}, rejected {n["sh_rej"]}: '
                      + ', '.join(f'{np.mod(b, L):.2f}: {100 * (c - b):+.0f} cm{"" if ok else " (rej)"}'
                                  for _, b, c, ok, _ in run.sh_rows))
            ra = reacq(rows, L)
            if ra:
                e = np.array([p - c for _, p, c, _ in ra])
                print(f'  re-acquisitions after > 0.5 m without a fit: {len(ra)}, |error| mean {100 * np.mean(np.abs(e)):.1f} cm, '
                      f'max {100 * np.max(np.abs(e)):.1f}: ' + ', '.join(f'{a:.2f}: {100 * p:+.1f}/{100 * c:+.1f}' + ('' if u == 'ok' else ' rej')
                                                                     for a, p, c, u in ra))
            gr = [g for g in gps_rows if g[2]]
            if gr:
                print(f'  last GPS used at s = {gr[-1][1]:.2f}; '
                      f'rejected while s mod L in 4.4-10: {sum(1 for g in gps_rows if not g[2] and 4.4 < np.mod(g[1], L) < 10)}')

    print(f'\nheld-out test: camera hidden over {hold_len} m windows, prediction vs the next camera fit')
    for mode in ('full', 'white', 'along', 'none'):
        H = held_out(vc, route, odo, gps, cam, results['none'][0], mode)
        for lo, hi in hold_ranges:
            m = (H[:, 0] >= lo) & (H[:, 0] < hi)
            if m.any():
                e = H[m, 1]
                print(f'  [{mode:5s}] s {lo}-{hi}: {m.sum()} windows, error mean {100 * e.mean():+.1f} cm, '
                      f'rms {100 * np.sqrt(np.mean(e ** 2)):.1f}, max {100 * np.abs(e).max():.1f}; '
                      f'predicted sd {100 * H[m, 2].mean():.1f} cm, rms(error/sd) {np.sqrt(np.mean((e / H[m, 2]) ** 2)):.2f}')

    old = sys.argv[4] if len(sys.argv) > 4 else None
    tag = os.path.basename(old)[len('c3_replay_'):-4] if old else ''
    sfx = ('' if OPT['dr'] == 'old' else f"_axle_{OPT['enc']}") + ('_shift' if OPT['shift'] is not None else '')
    base = os.path.join(os.path.dirname(odo_file), 'fusion_replay' + ('_' + tag if tag else '') + sfx)
    rows = results['full'][0]
    with open(base + '.csv', 'w') as fh:
        fh.write('t,s,d_cm,h_deg,h_exp_deg,sd_s_cm,sd_d_cm,sd_h_deg,src,used,cam_d_cm\n')
        for r in rows:
            fh.write(f"{r['t']:.3f},{r['s']:.3f},{100 * r['d']:.2f},{np.degrees(r['h']):.2f},{np.degrees(r['h_exp']):.2f},"
                     f"{100 * r['sd_s']:.2f},{100 * r['sd_d']:.2f},{np.degrees(r['sd_h']):.2f},{r['src']},{r['used']},"
                     f"{100 * r['cam_d']:.2f}\n")
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(3, 1, figsize=(12, 8), sharex=True)
    for mode, c in (('full', 'C0'), ('white', 'C3'), ('along', 'C1'), ('none', 'C2')):
        R = results[mode][0]
        S = np.array([r['s'] for r in R])
        D = 100 * np.array([r['d'] for r in R])
        sdd = 100 * np.array([r['sd_d'] for r in R])
        ax[0].plot(S, D, color=c, lw=1, label=f'fusion, GPS {mode}')
        ax[1].plot(S, sdd, color=c, lw=1, label=mode)
        ax[2].plot(S, 100 * np.array([r['sd_s'] for r in R]), color=c, lw=1, label=mode)
    S = np.array([r['s'] for r in rows])
    for u, c, lab in (('ok', 'k', 'camera fit used'), ('rej', 'r', 'camera fit rejected')):
        m = np.array([r['used'] == u for r in rows])
        ax[0].plot(S[m], 100 * np.array([r['cam_d'] for r in rows])[m], '.', color=c, ms=3, label=lab)
    if old and os.path.exists(old):
        O = np.genfromtxt(old, delimiter=',', names=True, dtype=None, encoding=None)
        ax[0].plot(O['s'], O['d_cm'], 'm--', lw=0.8, label='C3 estimator (c3_replay)')
    ax[1].axhline(100 * sd_stop, color='r', lw=0.8, ls=':')
    ax[0].set_ylabel('offset (cm, + left)')
    ax[1].set_ylabel('offset sd (cm)')
    ax[2].set_ylabel('along-route sd (cm)')
    ax[2].set_xlabel('front axle position along the route s (m, unwrapped)')
    for a in ax:
        a.legend(fontsize=7, ncol=4)
        a.grid(alpha=0.3)
    ax[0].set_ylim(-15, 15)
    fig.tight_layout()
    fig.savefig(base + '.png', dpi=90)
    print(f'-> {base}.csv/.png')


if __name__ == '__main__':
    main()
