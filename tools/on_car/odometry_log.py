"""
odometry_log.py

Plan step C2 (camera report §24): log the distance travelled and the turn
rate while the car is rolled by hand, so each raw camera frame can be given
the distance travelled since the start. No GPS, no lidar. Motors are held
at zero throttle and zero steering the whole time.

Run on the QCar, alongside lane_camera.py (start this one second):
    python -u odometry_log.py
Stop with Ctrl+C. Output: odometry.csv, one row every --period s:
    t_epoch      same clock as lane_camera.csv and the raw frame names
    encoder      raw motor encoder counts
    dist_m       distance from the encoder counts (encoder * CPS_TO_MPS)
    tach_mps     speed from the tachometer
    dist_tach_m  integral of tach_mps
    gyro_z       turn rate (rad/s, + = left)
    heading_deg  integral of gyro_z since the start (gyro bias removed, see below)
The gyro bias is averaged over the first --bias-time s (keep the car still).
"""
import argparse
import time
import numpy as np
from pal.products.qcar import QCar


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--period', type=float, default=0.02, help='s between log rows')
    ap.add_argument('--bias-time', type=float, default=3.0, help='s to average the gyro bias (car still)')
    ap.add_argument('--log', default='odometry.csv')
    args = ap.parse_args()

    qcar = QCar(readMode=1, frequency=200)
    cps_to_mps = getattr(qcar, 'CPS_TO_MPS', np.nan)
    print(f'CPS_TO_MPS = {cps_to_mps}')
    with qcar, open(args.log, 'w') as f:
        f.write('t_epoch,encoder,dist_m,tach_mps,dist_tach_m,gyro_z,heading_deg\n')
        # Gyro bias while still
        print(f'keep the car still for {args.bias_time:.0f} s (gyro bias) ...')
        g, t_end = [], time.time() + args.bias_time
        while time.time() < t_end:
            qcar.read()
            qcar.write(0, 0)
            g.append(float(qcar.gyroscope[2]))
            time.sleep(0.005)
        bias = float(np.mean(g))
        qcar.read()
        enc0 = int(qcar.motorEncoder[0])
        print(f'gyro bias {bias:+.5f} rad/s; logging, roll the car now (Ctrl+C to stop)')
        heading = dist_tach = 0.0
        t_prev = last_print = time.time()
        try:
            while True:
                qcar.read()
                qcar.write(0, 0)
                now = time.time()
                dt, t_prev = now - t_prev, now
                tach = float(qcar.motorTach[0]) if np.ndim(qcar.motorTach) else float(qcar.motorTach)
                gz = float(qcar.gyroscope[2]) - bias
                dist_tach += tach * dt
                heading += gz * dt
                enc = int(qcar.motorEncoder[0]) - enc0
                f.write(f'{now:.3f},{enc},{enc * cps_to_mps:.4f},{tach:.4f},{dist_tach:.4f},'
                        f'{gz:.5f},{np.degrees(heading):.2f}\n')
                if now - last_print >= 1.0:
                    print(f'dist={enc * cps_to_mps:6.3f} m (tach {dist_tach:6.3f} m)  heading={np.degrees(heading):+6.1f} deg')
                    last_print = now
                time.sleep(max(0.0, args.period - (time.time() - now)))
        except KeyboardInterrupt:
            pass
    print(f'logged to {args.log}')


if __name__ == '__main__':
    main()
