"""
shift_meas.py  (PC only)

Camera report §61: along-route position measurements from the front camera for a
recorded drive, for fusion_replay.py --shift. Runs lane_camera.shift_fit (the same
coarse-to-fine fit as on the car) on every saved front frame taken while moving where
the map curvature changes within the view, with the profile at the car's logged s at
the frame time. The measurement is s_true = logged s + best shift (independent of the
estimator that later uses it).

    python tools/checks/shift_meas.py <raw frames folder> <odometry_drive.csv>
Output: shift_meas.csv next to the odometry file:
    t_epoch, s_logged, sh_m, s_true, rms0, rms, ok   (ok = lane_camera's sh_ok rule)
"""
import os
import sys
import numpy as np
import cv2

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
import common  # noqa: E402  qcar/, tools/ folders and ../libraries/python on sys.path
import lane_camera as lc
from common import load_vc


def main():
    folder, odo_file = sys.argv[1].rstrip('/\\'), sys.argv[2]
    vc = load_vc()
    from hal.products.mats import SDCSRoadMap
    route = vc['RoutePath'](vc['remove_backward_points'](
        SDCSRoadMap(leftHandTraffic=False).generate_path(vc['nodeSequence'])))
    o = np.genfromtxt(odo_file, delimiter=',', names=True, dtype=None, encoding=None)
    frames = sorted(float(f[:-10]) for f in os.listdir(folder) if f.endswith('_front.jpg'))
    frames = [t for t in frames if o['t_epoch'][0] <= t <= o['t_epoch'][-1]]
    S = np.interp(frames, o['t_epoch'], o['cs_s'])
    D = np.interp(frames, o['t_epoch'], o['dist_m'])
    win = (lc.PROFILE_S >= 0) & (lc.PROFILE_S <= lc.MF_FIT_MAX)
    out = os.path.join(os.path.dirname(odo_file), 'shift_meas.csv')
    n = n_ok = 0
    with open(out, 'w') as f:
        f.write('t_epoch,s_logged,sh_m,s_true,rms0,rms,ok\n')
        prevD = None
        for t, s, dd in zip(frames, S, D):
            moving = prevD is not None and dd - prevD > 0.005
            prevD = dd
            k = route.curvature(s + lc.PROFILE_S)
            if not moving or np.ptp(k[win]) < lc.SHIFT_MIN_DK:
                continue
            _, dbg = lc.process(cv2.imread(os.path.join(folder, f'{t:.3f}_front.jpg')))
            r = lc.shift_fit(dbg, k)
            if not np.isfinite(r['sh_rms0']):
                continue
            n += 1
            n_ok += r['sh_ok']
            f.write(f"{t:.3f},{s:.4f},{r['sh_m']:.3f},{s + r['sh_m']:.4f},{r['sh_rms0']:.4f},"
                    f"{r['sh_rms']:.4f},{int(r['sh_ok'])}\n")
    print(f'{n} frames fitted, {n_ok} usable -> {out}')


if __name__ == '__main__':
    main()
