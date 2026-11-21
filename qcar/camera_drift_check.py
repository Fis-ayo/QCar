"""
camera_drift_check.py  (PC only)

Camera report §67: has the front camera's mounting (pitch, height) moved since the
checkerboard calibration of 2026-11-18 (§27-§30)? On recorded drives:
  1. lane width measured on straight 'both' frames (lane_camera.csv; true 25.7 cm);
  2. pitch from parallelism: on straight frames (4 -> 14) the yellow line and the road
     edge are parallel only at the right pitch; the camera height then follows from
     the lane width (width scales with the height);
  3. curvature on the 2 -> 4 arc (map 1.12 /m) with lane_camera's values and with the
     fitted pitch/height: if the fitted values remove the ~16 % (§64), pitch is the cause.

    python qcar/camera_drift_check.py
"""
import os
import sys
import numpy as np
import cv2

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, '..', '..', 'libraries', 'python'))
import lane_camera as lc
from c3_replay import load_vc

A = os.path.join(HERE, '..', 'logs', 'car A')
B = os.path.join(HERE, '..', 'logs', 'car B')
DRIVES = [  # name, folder with odometry_drive.csv + lane_camera.csv, raw frames
    ('A 11-18 (§40)', A, os.path.join(A, 'raw_frames', '20261118_234715')),
    ('A 11-19 (§46)', os.path.join(A, 'Fresh_Log'), os.path.join(A, 'Fresh_Log', '20261119_234304')),
    ('A drive A', os.path.join(A, '20261120_driveA'), os.path.join(A, '20261120_driveA', '20261120_190909')),
    ('A drive C', os.path.join(A, '20261120_driveC'), os.path.join(A, '20261120_driveC', '20261120_194525')),
    ('A drive E', os.path.join(A, '20261120_driveE'), os.path.join(A, '20261120_driveE', '20261120_215026')),
    ('A drive F', os.path.join(A, '20261120_driveF'), os.path.join(A, '20261120_driveF', '20261120_223328')),
    ('B drive A', os.path.join(B, '20261120_driveA'), os.path.join(B, '20261120_driveA', '20261120_220828')),
]
PITCHES = np.radians(np.round(np.arange(-3.0, 2.01, 0.1), 2))
KS = np.round(np.arange(0.6, 2.41, 0.05), 2)
P0, H0 = lc.PITCH, lc.CAM_H


def set_pitch_h(p, h):
    lc.PITCH, lc.CAM_H = p, h
    lc.HORIZON_V = lc.CY - lc.FY * np.tan(p)


def line_fit(p):
    """Robust line l = a*f + b through floor points p (forward, left)."""
    if len(p) < lc.MIN_POINTS // 2:
        return None
    keep = np.ones(len(p), bool)
    for _ in range(3):
        a, b = np.polyfit(p[keep, 0], p[keep, 1], 1)
        r = p[:, 1] - (a * p[:, 0] + b)
        keep = np.abs(r) < max(3 * np.std(r[keep]), 0.004)
        if keep.sum() < 10:
            return None
    return a, b


def main():
    vc = load_vc()
    from hal.products.mats import SDCSRoadMap
    route = vc['RoutePath'](vc['remove_backward_points'](
        SDCSRoadMap(leftHandTraffic=False).generate_path(vc['nodeSequence'])))
    RL = route.L
    print(f'lane_camera now: pitch {np.degrees(P0):+.2f} deg, height {100 * H0:.1f} cm, ahead {100 * lc.CAM_AHEAD:.1f} cm')
    print(f'{"drive":15s} {"width log":>9s} {"pitch fit":>9s} {"sd":>5s} {"height fit":>10s} {"frames":>6s} | '
          f'2->4 arc k: {"now":>5s} {"fitted":>6s} (map 1.12)')
    for name, folder, frames in DRIVES:
        o = np.genfromtxt(os.path.join(folder, 'odometry_drive.csv'), delimiter=',', names=True, dtype=None, encoding=None)
        cam = np.genfromtxt(os.path.join(folder, 'lane_camera.csv'), delimiter=',', names=True, dtype=None, encoding=None)
        # 1. lane width in the log, straight 'both' frames
        sc = np.mod(np.interp(cam['t_epoch'], o['t_epoch'], o['cs_s']), RL)
        mw = (cam['mode'] == 'both') & (sc > 4.6) & (sc < 7.0) & np.isfinite(cam['width_cm'])
        width_log = np.median(cam['width_cm'][mw]) if mw.any() else np.nan
        # frames on the straight and on the 2 -> 4 arc
        fr = sorted(float(f[:-10]) for f in os.listdir(frames) if f.endswith('_front.jpg'))
        S = np.interp(fr, o['t_epoch'], o['cs_s'])
        first = S - o['cs_s'][0] < 12
        st = [t for t, s, f1 in zip(fr, S, first) if f1 and 4.6 < s % RL < 7.0]
        arc = [(t, s) for t, s, f1 in zip(fr, S, first) if f1 and 2.27 < s % RL < 2.92]
        # 2. pitch from parallelism, height from width
        pitches, widths = [], []
        for t in st:
            set_pitch_h(P0, H0)
            out, dbg = lc.process(cv2.imread(os.path.join(frames, f'{t:.3f}_front.jpg')))
            if out['mode'] != 'both':
                continue
            best = None
            for p in PITCHES:
                set_pitch_h(p, H0)
                y, e = lc.boundary_points(dbg)
                fy, fe = line_fit(y), line_fit(e)
                if fy is None or fe is None:
                    continue
                diff = abs(fy[0] - fe[0])
                if best is None or diff < best[0]:
                    w = ((fy[1] + fy[0] * 0.3) - (fe[1] + fe[0] * 0.3)) * np.cos(np.arctan(fy[0]))
                    best = (diff, p, w)
            if best is not None and best[0] < 0.01:
                pitches.append(best[1])
                widths.append(best[2])
        if not pitches:
            print(f'{name:15s} {width_log:9.1f}  no usable straight frames')
            continue
        pf = float(np.median(pitches))
        hf = H0 * lc.LANE_W / float(np.median(widths))
        # 3. curvature on the 2 -> 4 arc, now vs fitted
        res = []
        for p, h in ((P0, H0), (pf, hf)):
            tot = np.zeros(len(KS))
            n = 0
            for t, s in arc:
                set_pitch_h(P0, H0)
                out, dbg = lc.process(cv2.imread(os.path.join(frames, f'{t:.3f}_front.jpg')))
                set_pitch_h(p, h)
                y, e = lc.boundary_points(dbg)
                r = []
                for kk in KS:
                    c, nn = lc.lane_from_profile(np.full(len(lc.PROFILE_S), kk))
                    acc = m = 0
                    for pts, side in ((y, lc.LANE_W / 2), (e, -lc.LANE_W / 2)):
                        if len(pts) < lc.MIN_POINTS // 2:
                            continue
                        if len(pts) > lc.MF_MAX_PTS:
                            pts = pts[np.linspace(0, len(pts) - 1, lc.MF_MAX_PTS).astype(int)]
                        dd, hh, rr = lc.fit_boundary(pts, c, nn, side)
                        if abs(dd) < lc.MF_MAX_D and abs(hh) < lc.MF_MAX_H:
                            acc += rr ** 2 * len(pts)
                            m += len(pts)
                    r.append(np.sqrt(acc / m) if m else np.nan)
                r = np.array(r)
                if np.all(np.isnan(r)) or np.nanmin(r) > 0.3:
                    continue
                tot += np.nan_to_num(r ** 2, nan=1.0)
                n += 1
            res.append(KS[np.argmin(tot)] if n else np.nan)
        set_pitch_h(P0, H0)
        print(f'{name:15s} {width_log:9.1f} {np.degrees(pf):+9.2f} {np.degrees(np.std(pitches)):5.2f} '
              f'{100 * hf:9.1f}  {len(pitches):6d} | {res[0]:11.2f} {res[1]:6.2f}', flush=True)


if __name__ == '__main__':
    main()
