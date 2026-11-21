"""
calib_capture.py

Camera report §26/§27: frames for a checkerboard calibration of the FRONT CSI
camera at 820x410 (the fisheye numbers in lane_camera.py are borrowed and don't
fit this camera). Works over SSH, no screen needed. It only saves frames; the
checkerboard detection and the calibration are done on the PC.

Run on the QCar:
    python -u calib_capture.py                 # 60 s, one frame every 1 s
    python -u calib_capture.py --duration 90 --every 0.7
While it runs, hold the printed checkerboard (flat, on card or a board) in
front of the front camera and move it slowly: near and far, left, right,
top, bottom and the corners of the view, tilted a bit in different
directions. Keep the whole board in view in each position.
It prints whether OpenCV can see the board in each saved frame ("board: yes").
Output: calib_frames/<start time>/<t_epoch>.png
"""
import argparse
import os
import time
import cv2


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--duration', type=float, default=60, help='s')
    ap.add_argument('--every', type=float, default=1.0, help='s between saved frames')
    ap.add_argument('--cols', type=int, default=9, help='inner corners per row of the board')
    ap.add_argument('--rows', type=int, default=6, help='inner corners per column of the board')
    args = ap.parse_args()

    from pal.products.qcar import QCarCameras
    cams = QCarCameras(frameWidth=820, frameHeight=410, frameRate=30, enableFront=True)
    out = os.path.join('calib_frames', time.strftime('%Y%m%d_%H%M%S'))
    os.makedirs(out)
    print(f'saving to {out}/ for {args.duration:.0f} s; looking for a {args.cols}x{args.rows} inner-corner board')
    t0 = last = time.time()
    n = n_ok = 0
    try:
        while time.time() - t0 < args.duration:
            cams.csiFront.read()
            img = cams.csiFront.imageData
            if img is None or not img.any() or time.time() - last < args.every:
                time.sleep(0.01)
                continue
            last = time.time()
            frame = img.copy()
            cv2.imwrite(os.path.join(out, f'{last:.3f}.png'), frame)
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            ok, _ = cv2.findChessboardCorners(gray, (args.cols, args.rows),
                                              cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK)
            n += 1
            n_ok += int(ok)
            print(f'{last - t0:5.1f} s  frame {n:3d}  board: {"yes" if ok else "no"}   ({n_ok} with board so far)')
    except KeyboardInterrupt:
        pass
    finally:
        cams.terminate()
    print(f'{n} frames saved, {n_ok} with the board visible -> {out}/')


if __name__ == '__main__':
    main()
