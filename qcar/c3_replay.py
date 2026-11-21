"""
c3_replay.py  (PC only)

Camera report §35: replays a recorded drive through the camera-steering (C3)
estimator of vehicle_control.py, without the car: position along the route
(encoder x enc_scale + GPS along-route correction), lane_camera.map_fit() on the
recorded front frames with the curvature profile at that position, and the
encoder/gyro dead reckoning in between. The steering itself isn't simulated
(the recorded car was steered by the table), so this checks the estimator only.

RoutePath, remove_backward_points and the cs_* settings are taken from
vehicle_control.py's source, so the replay uses the same code and numbers.

    python qcar/c3_replay.py "logs/car A/raw_frames/20261118_222611" "logs/car A/odometry_drive.csv" "logs/car A/vehicle_control.log"
    python qcar/c3_replay.py <frames> <odometry_drive.csv> <vehicle_control.log> "logs/car A/lane_camera.csv"
The optional 4th argument replays the map fits the car actually received (mf_* in
lane_camera.csv, every processed frame, ~13/s) instead of recomputing them on the
saved frames (5/s). Use it for a camera-steering drive, to test the acceptance rules
on the real message stream (the fits come from the lane_camera.py of that drive).
Output: c3_replay_<folder>.csv / .png next to the frames folder.

Dead reckoning (report §53): --dr old (default, the car's current model: carries the
rear axle's offset as the front's) or --dr axle (rear axle carried along the body, front
axle by geometry, lane_fusion.axle_step; offline test only, not validated, §54);
--enc front|rear|mean: which axle the encoder distance is (axle model only). The axle
model's outputs get a suffix, e.g. c3_replay_<folder>_axle_front.
--shift 0.25|0.5|0.75|1 (frames mode only, i.e. without the lane_camera.csv argument):
along-route correction from the camera at curve starts/ends (lane_camera.shift_fit, §56),
s += gain x shift per frame where it is observable; suffix _shift<gain>.
"""
import ast
import os
import re
import sys
import numpy as np
import cv2

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, '..', '..', 'libraries', 'python'))
import lane_camera as lc
import lane_fusion as lf


def load_vc():
    """RoutePath, remove_backward_points and the settings from vehicle_control.py."""
    src = open(os.path.join(HERE, 'vehicle_control.py'), encoding='utf-8').read()
    tree = ast.parse(src)
    keep = []
    names = {'RoutePath', 'remove_backward_points'}
    for node in tree.body:
        if isinstance(node, (ast.ClassDef, ast.FunctionDef)) and node.name in names:
            keep.append(node)
        elif isinstance(node, ast.Assign) and len(node.targets) == 1 and isinstance(node.targets[0], ast.Name):
            n = node.targets[0].id
            if n.startswith(('cs_', 'enc_scale', 'gps_front_ahead', 'profile_s', 'nodeSequence',
                             'startDelay')):
                keep.append(node)
    ns = {'np': np}
    exec(compile(ast.Module(body=keep, type_ignores=[]), 'vehicle_control.py', 'exec'), ns)
    return ns


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def pop_opt(name, default, choices):
    """--name value from sys.argv (removed), or the default."""
    if name in sys.argv:
        k = sys.argv.index(name)
        val = sys.argv[k + 1]
        del sys.argv[k:k + 2]
        assert val in choices, f'{name} must be one of {choices}'
        return val
    return default


DR = pop_opt('--dr', 'old', ('axle', 'old')) if __name__ == '__main__' else 'old'
ENC = pop_opt('--enc', 'front', lf.ENC_AXLES) if __name__ == '__main__' else 'front'
SHIFT_GAIN = float(pop_opt('--shift', '0', ('0', '0.25', '0.5', '0.75', '1'))) if __name__ == '__main__' else 0.0


def main():
    folder, odo_file, log_file = sys.argv[1].rstrip('/\\'), sys.argv[2], sys.argv[3]
    vc = load_vc()
    from hal.products.mats import SDCSRoadMap
    rm = SDCSRoadMap(leftHandTraffic=False)
    route = vc['RoutePath'](vc['remove_backward_points'](rm.generate_path(vc['nodeSequence'])))
    assert np.allclose(vc['profile_s'], lc.PROFILE_S), 'profile_s differs from lane_camera.PROFILE_S'

    odo = np.genfromtxt(odo_file, delimiter=',', names=True)
    cps = odo['dist_m'][-1] / odo['encoder'][-1]
    gps = []
    for line in open(log_file):
        m = re.match(r'GPS(?: rejected)?: \(([-\d.]+),([-\d.]+),([-\d]+)deg\).*t_epoch=([\d.]+)', line)
        if m:
            gps.append((float(m.group(4)), float(m.group(1)), float(m.group(2)), np.radians(float(m.group(3)))))
    gps = np.array(gps)
    logged = None
    if len(sys.argv) > 4:
        L = np.genfromtxt(sys.argv[4], delimiter=',', names=True, dtype=None, encoding=None)
        logged = {float(r['t_epoch']): r for r in L}
        frames = sorted(logged)
    else:
        frames = sorted(float(f[:-10]) for f in os.listdir(folder) if f.endswith('_front.jpg'))
    frames = [t for t in frames if odo['t_epoch'][0] <= t <= odo['t_epoch'][-1]]

    # start: first GPS reading, as vehicle_control does
    g0 = gps[gps[:, 0] >= odo['t_epoch'][0] - 5][0]
    sG0, lat0 = route.locate(g0[1], g0[2], g0[3], vc['cs_start_head'])
    s = sG0 + vc['gps_front_ahead']
    print(f'start: GPS ({g0[1]:.2f},{g0[2]:.2f}) -> s = {s:+.3f} m (front axle), {100 * lat0:+.1f} cm from the route')

    start_delay = vc['startDelay']
    t_move0 = odo['t_epoch'][0]
    moving = np.nonzero(odo['dist_m'] > 0.002)[0]
    bias_rows = odo['t_epoch'] < (odo['t_epoch'][moving[0]] if len(moving) else odo['t_epoch'][-1])
    bias = float(np.mean(odo['gyro_z_raw'][bias_rows][:max(1, int(start_delay / 0.02))]))
    d = h = 0.0
    psi = None      # body heading (axle model); None = take it from h at the next step
    h_exp_prev = 0.0
    fit_s = s
    rows = []
    gi = 0
    fi = 0
    n_gps = n_fit = n_rej = 0
    rej_s, rej_vals = None, []
    shifts = []
    pend, n_wait = None, 0
    enc_prev = odo['encoder'][0]
    for i in range(len(odo)):
        t = odo['t_epoch'][i]
        dt = t - odo['t_epoch'][i - 1] if i else 0.0
        ds = vc['enc_scale'] * (odo['encoder'][i] - enc_prev) * cps
        enc_prev = odo['encoder'][i]
        dpsi = (odo['gyro_z_raw'][i] - bias) * dt
        if DR == 'old':
            s_prev = s
            s += ds
            th_path = float(route.heading(s))
            h_exp = float(wrap(float(route.heading(s - vc['cs_wheelbase'])) - th_path))
            d += np.sin(h - h_exp_prev) * ds
            h = float(wrap(h + dpsi - wrap(th_path - float(route.heading(s_prev)))))
            h_exp_prev = h_exp
        else:   # §53: rear axle carried, front axle by geometry
            if psi is None:
                psi = h + float(route.heading(s))
            s, d, psi = lf.axle_step(route, vc['cs_wheelbase'], s, d, psi, ds, dpsi, ENC)
            th_path = float(route.heading(s))
            h = float(wrap(psi - th_path))
            h_exp = float(wrap(float(route.heading(s - vc['cs_wheelbase'])) - th_path))
        # GPS readings up to now
        while gi < len(gps) and gps[gi, 0] <= t:
            sG, latG = route.project(gps[gi, 1], gps[gi, 2], s - vc['gps_front_ahead'], 0.4)
            innov = sG + vc['gps_front_ahead'] - s
            if abs(innov) < vc['cs_gps_gate'] and abs(latG) < vc['cs_gps_lat_gate']:
                s += vc['cs_gps_gain'] * innov
                n_gps += 1
            gi += 1
        # camera frames up to now
        src = None
        while fi < len(frames) and frames[fi] <= t:
            if logged is not None:
                r = logged[frames[fi]]
                out = dict(mode=str(r['mode']), offset_cm=float(r['offset_cm']), heading_deg=float(r['heading_deg']))
                mf = dict(mf_src=str(r['mf_src']), mf_d_cm=float(r['mf_d_cm']), mf_h_deg=float(r['mf_h_deg']),
                          mf_rms_cm=float(r['mf_rms_cm']))
            else:
                bgr = cv2.imread(os.path.join(folder, f'{frames[fi]:.3f}_front.jpg'))
                out, dbg = lc.process(bgr)
                if SHIFT_GAIN > 0 and odo['dist_m'][i] > 0.002:
                    # §56: along-route correction from the camera at curve starts / ends
                    sf = lc.shift_fit(dbg, route.curvature(s + vc['profile_s']))
                    if sf['sh_ok']:
                        ds_cam = SHIFT_GAIN * sf['sh_m']
                        if DR == 'old':      # keep the gyro heading: heading to the lane changes with s
                            h = float(wrap(h - (float(route.heading(s + ds_cam)) - float(route.heading(s)))))
                        s += ds_cam
                        shifts.append((s, sf['sh_m'], sf['sh_rms0'], sf['sh_rms']))
                mf = lc.map_fit(dbg, route.curvature(s + vc['profile_s']))
            src = mf['mf_src']
            accept = False
            parked = odo['dist_m'][i] < 0.002
            head_bad = src in ('YE', 'E', 'Y') and abs(wrap(np.radians(mf['mf_h_deg']) - h_exp)) > vc['cs_max_head_fit']
            if src not in ('YE', 'E', 'Y') or head_bad:
                pend = None
                if head_bad:
                    src = 'hbad'
            elif (not parked and s - fit_s > vc['cs_confirm_after']
                  and (pend is None or abs(mf['mf_d_cm'] / 100 - pend) > vc['cs_confirm_tol'])):
                pend = mf['mf_d_cm'] / 100
                n_wait += 1
                print(f'  waiting for confirmation at s = {s:.2f}: camera {mf["mf_d_cm"]:+.1f} cm ({src}), predicted {100 * d:+.1f}')
                src = 'wait'
            else:
                pend = None
                md = mf['mf_d_cm'] / 100
                gate = vc['cs_gate0'] + vc['cs_gate_per_m'] * (s - fit_s)
                if odo['dist_m'][i] < 0.002 or abs(md - d) <= gate:
                    accept = True
                else:
                    if rej_s is None:
                        rej_s, rej_vals = s, []
                    rej_vals.append(md)
                    n_rej += 1
                    if ((s - rej_s > vc['cs_relock_dist'] and np.std(rej_vals[-5:]) < vc['cs_relock_sd'])
                            or (len(rej_vals) >= vc['cs_relock_n'] and np.std(rej_vals[-vc['cs_relock_n']:]) < vc['cs_relock_n_sd'])):
                        accept = True
                        print(f'  re-lock at s = {s:.2f}: {100 * d:+.1f} -> {100 * md:+.1f} cm')
                    else:
                        print(f'  ignored at s = {s:.2f}: camera {100 * md:+.1f} cm, predicted {100 * d:+.1f}, gate {100 * gate:.1f}')
                        src = 'rej'
            if accept:
                rej_s = None
                d = mf['mf_d_cm'] / 100
                k_win = route.curvature(s + np.linspace(vc['cs_head_window'][0], vc['cs_head_window'][1], 37))
                if (np.ptp(k_win) <= vc['cs_head_kchange'] and mf['mf_rms_cm'] < vc['cs_head_rms']
                        and abs(wrap(np.radians(mf['mf_h_deg']) - h_exp)) < vc['cs_max_head']):
                    h = np.radians(mf['mf_h_deg'])
                    psi = None
                fit_s = s
                n_fit += 1
            elif out['mode'] == 'both' and odo['dist_m'][i] < 0.002:
                d, h = out['offset_cm'] / 100, np.radians(out['heading_deg'])
                psi = None
                fit_s, src = s, 'both'
            rows.append(dict(t=frames[fi], s=s, d=100 * d, h=np.degrees(h), h_exp=np.degrees(h_exp),
                             src=src if src else 'none', dr=s - fit_s))
            fi += 1
        if odo['dist_m'][i] > 0.002:
            if s - fit_s > vc['cs_dr_max']:
                print(f'would STOP at t={t:.2f}: no camera fit for {s - fit_s:.2f} m (s = {s:.2f})')
                break
            if abs(d) > vc['cs_max_offset']:
                print(f'would STOP at t={t:.2f}: offset {100 * d:+.1f} cm (s = {s:.2f})')
                break

    D = np.array([r['d'] for r in rows])
    print(f'{len(rows)} frames, {n_fit} camera fits, {n_wait} waiting for confirmation, {n_rej} ignored by the jump gate, '
          f'{n_gps} GPS along-route corrections')
    print(f'offset: mean {D.mean():+.1f} cm, sd {D.std():.1f}, max |d| {np.abs(D).max():.1f}; '
          f'longest stretch without a fit {max(r["dr"] for r in rows):.2f} m')
    print(f'dead reckoning: {DR}' + (f', encoder distance = {ENC} axle' if DR == 'axle' else ''))
    if SHIFT_GAIN > 0:
        print(f'camera along-route corrections (gain {SHIFT_GAIN}): {len(shifts)}')
        for sc, sh, r0, rb in shifts:
            print(f'  s = {sc:7.3f} (mod {np.mod(sc, route.L):5.2f}): shift {sh:+.2f} m, rms {r0:.3f} -> {rb:.3f}')
    out = os.path.join(os.path.dirname(folder), 'c3_replay_' + os.path.basename(folder)
                       + ('' if DR == 'old' else f'_axle_{ENC}') + (f'_shift{SHIFT_GAIN:g}' if SHIFT_GAIN > 0 else ''))
    with open(out + '.csv', 'w') as f:
        f.write('t,s,d_cm,h_deg,h_exp_deg,src,dr_m\n')
        for r in rows:
            f.write(f"{r['t']:.3f},{r['s']:.3f},{r['d']:.2f},{r['h']:.2f},{r['h_exp']:.2f},{r['src']},{r['dr']:.3f}\n")
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    S = np.array([r['s'] for r in rows])
    fig, ax = plt.subplots(2, 1, figsize=(11, 6), sharex=True)
    for k, c in (('YE', 'green'), ('E', 'blue'), ('Y', 'orange'), ('both', 'black')):
        m = np.array([r['src'] == k for r in rows])
        ax[0].plot(S[m], D[m], '.', color=c, label=k)
        ax[1].plot(S[m], np.array([r['h'] for r in rows])[m], '.', color=c)
    m = np.array([r['src'] not in ('YE', 'E', 'Y', 'both') for r in rows])
    ax[0].plot(S[m], D[m], '.', color='red', label='DR')
    ax[1].plot(S[m], np.array([r['h'] for r in rows])[m], '.', color='red')
    ax[1].plot(S, [r['h_exp'] for r in rows], 'k--', lw=0.8, label='expected')
    ax[0].axhline(0, color='k', lw=0.5)
    ax[0].set_ylabel('offset (cm, + left)')
    ax[0].legend(fontsize=7, ncol=5)
    ax[1].set_ylabel('heading to lane (deg)')
    ax[1].set_xlabel('front axle position along the route s (m, node 0 = 0)')
    ax[1].legend(fontsize=7)
    fig.tight_layout()
    fig.savefig(out + '.png', dpi=90)
    print(f'-> {out}.csv/.png')


if __name__ == '__main__':
    main()
