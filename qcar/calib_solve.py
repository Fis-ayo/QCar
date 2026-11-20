"""
calib_solve.py  (PC only)

Camera report §26: checkerboard calibration of the FRONT CSI camera at 820x410,
from the frames saved by calib_capture.py.

    python qcar/calib_solve.py "logs/car A/calib_frames/20261118_203057"
    python qcar/calib_solve.py <folder> --cols 7 --rows 4 --square 0.034
    python qcar/calib_solve.py <folder1> <folder2>      # several recordings combined

Steps:
  1. Find the board's inner corners in every frame (classic detector), refined
     to sub-pixel. The SB detector is off by default (--sb): on the 3.4 cm
     letter board (full squares plus half squares at the edges) it sometimes
     picks a wrong grid that skips a row of squares (report §28).
  2. Standard OpenCV model (K + 5 distortion coefficients, the same model
     lane_camera.py uses), solved with cv2.calibrateCamera. Frames with a
     reprojection error well above the rest are dropped and it is solved again.
  3. For comparison, OpenCV's fisheye model (K + 4 coefficients).
Output, in <folder>/../calib_result_<folder name(s)>/:
  calib_result.json       K, DIST, errors, frames used, for both models
  per_frame.csv           reprojection error per frame
  coverage.png            all detected corners on one image (should fill the view)
  undist_<frame>.png      raw | borrowed intrinsics | new standard | new fisheye
"""
import argparse
import glob
import json
import os
import numpy as np
import cv2

IMG_W, IMG_H = 820, 410
# Borrowed values currently in lane_camera.py, for the side-by-side images.
K_OLD = np.array([[495.84, 0.00, 408.03],
                  [0.00, 454.60, 181.21],
                  [0.00, 0.00, 1.00]])
DIST_OLD = np.array([-0.57513, 0.37175, -0.00489, -0.00277, -0.11136])


def find_corners(gray, size, use_sb=False):
    """Inner corners (N,1,2) float32, or None. size = (cols, rows)."""
    flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
    ok, c = cv2.findChessboardCorners(gray, size, flags)
    if ok:
        crit = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 50, 0.01)
        c = cv2.cornerSubPix(gray, c, (5, 5), (-1, -1), crit)
        return c, 'classic'
    if not use_sb:
        return None, None
    ok, c = cv2.findChessboardCornersSB(gray, size, cv2.CALIB_CB_EXHAUSTIVE + cv2.CALIB_CB_ACCURACY)
    if ok:
        return c.astype(np.float32), 'sb'
    return None, None


def per_view_errors(obj, img, rvecs, tvecs, K, D, fisheye=False):
    errs = []
    for o, i, r, t in zip(obj, img, rvecs, tvecs):
        if fisheye:
            p, _ = cv2.fisheye.projectPoints(o.reshape(-1, 1, 3), r, t, K, D)
        else:
            p, _ = cv2.projectPoints(o, r, t, K, D)
        errs.append(float(np.sqrt(np.mean(np.sum((p.reshape(-1, 2) - i.reshape(-1, 2)) ** 2, axis=1)))))
    return np.array(errs)


def solve_standard(obj, img):
    rms, K, D, rv, tv = cv2.calibrateCamera(obj, img, (IMG_W, IMG_H), None, None)
    return rms, K, D.ravel(), rv, tv


def solve_fisheye(obj, img):
    obj_f = [o.reshape(-1, 1, 3).astype(np.float64) for o in obj]
    img_f = [i.reshape(-1, 1, 2).astype(np.float64) for i in img]
    flags = (cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC | cv2.fisheye.CALIB_FIX_SKEW)
    crit = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-7)
    rms, K, D, rv, tv = cv2.fisheye.calibrate(obj_f, img_f, (IMG_W, IMG_H), None, None,
                                              flags=flags, criteria=crit)
    return rms, K, D.ravel(), rv, tv


def solve_with_pruning(solver, obj, img, names, fisheye, max_rounds=5):
    """Solve, drop frames > max(2x median, 1 px), repeat until none are dropped."""
    keep = list(range(len(obj)))
    for _ in range(max_rounds):
        o = [obj[k] for k in keep]
        i = [img[k] for k in keep]
        rms, K, D, rv, tv = solver(o, i)
        e = per_view_errors(o, i, rv, tv, K, D, fisheye)
        lim = max(2.0 * np.median(e), 1.0)
        bad = [keep[j] for j in range(len(keep)) if e[j] > lim]
        if not bad or len(keep) - len(bad) < 10:
            break
        print(f'  dropping {len(bad)} frame(s) above {lim:.2f} px: '
              + ', '.join(names[b] for b in bad))
        keep = [k for k in keep if k not in bad]
    errs = {names[keep[j]]: float(e[j]) for j in range(len(keep))}
    return rms, K, D, keep, errs


def undistort_std(img, K, D):
    m1, m2 = cv2.initUndistortRectifyMap(K, D, None, K, (IMG_W, IMG_H), cv2.CV_16SC2)
    return cv2.remap(img, m1, m2, cv2.INTER_LINEAR)


def undistort_fish(img, K, D):
    # Keep the same focal length; balance 0 crops to valid pixels.
    m1, m2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), K, (IMG_W, IMG_H), cv2.CV_16SC2)
    return cv2.remap(img, m1, m2, cv2.INTER_LINEAR)


def label(img, text):
    out = img.copy()
    cv2.putText(out, text, (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 4)
    cv2.putText(out, text, (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('folders', nargs='+')
    ap.add_argument('--cols', type=int, default=7, help='inner corners per row')
    ap.add_argument('--rows', type=int, default=4, help='inner corners per column')
    ap.add_argument('--square', type=float, default=0.034, help='measured square size, m')
    ap.add_argument('--sb', action='store_true', help='also try the SB detector when the classic one fails')
    ap.add_argument('--n-examples', type=int, default=4, help='undistorted example images')
    args = ap.parse_args()

    size = (args.cols, args.rows)
    files = []
    for d in args.folders:
        files += sorted(glob.glob(os.path.join(d, '*.png')) + glob.glob(os.path.join(d, '*.jpg')))
    if not files:
        raise SystemExit(f'no images in {args.folders}')
    name = '+'.join(os.path.basename(os.path.normpath(d)) for d in args.folders)
    out = os.path.join(os.path.dirname(os.path.normpath(args.folders[0])), f'calib_result_{name}')
    os.makedirs(out, exist_ok=True)

    grid = np.zeros((args.cols * args.rows, 3), np.float32)
    grid[:, :2] = np.mgrid[0:args.cols, 0:args.rows].T.reshape(-1, 2) * args.square

    obj, img, names, frames = [], [], [], {}
    n_classic = n_sb = 0
    for f in files:
        im = cv2.imread(f)
        if im is None:
            continue
        if im.shape[1] != IMG_W or im.shape[0] != IMG_H:
            print(f'  skip {os.path.basename(f)}: {im.shape[1]}x{im.shape[0]}, expected {IMG_W}x{IMG_H}')
            continue
        c, how = find_corners(cv2.cvtColor(im, cv2.COLOR_BGR2GRAY), size, args.sb)
        if c is None:
            continue
        n_classic += how == 'classic'
        n_sb += how == 'sb'
        obj.append(grid.copy())
        img.append(c)
        names.append(os.path.basename(f))
        frames[names[-1]] = im
    print(f'{len(files)} images, board found in {len(obj)} ({n_classic} classic, {n_sb} SB detector)')
    if len(obj) < 10:
        raise SystemExit('fewer than 10 frames with the board: not enough to calibrate')

    # Coverage: where in the image the corners were seen.
    cov = frames[names[0]].copy() // 3
    for c in img:
        for p in c.reshape(-1, 2):
            cv2.circle(cov, (int(p[0]), int(p[1])), 2, (0, 255, 0), -1)
    cv2.imwrite(os.path.join(out, 'coverage.png'), cov)
    pts = np.concatenate([c.reshape(-1, 2) for c in img])
    cells = np.zeros((4, 4), int)
    for u, v in pts:
        cells[min(int(v / IMG_H * 4), 3), min(int(u / IMG_W * 4), 3)] += 1
    print('corner count per image region (4x4 grid, top row first):')
    for r in cells:
        print('   ' + ' '.join(f'{x:5d}' for x in r))

    print('\nstandard model (K + 5 coefficients, as lane_camera.py):')
    rms_s, K_s, D_s, keep_s, err_s = solve_with_pruning(solve_standard, obj, img, names, False)
    print(f'  {len(keep_s)} frames, rms {rms_s:.3f} px')
    print(f'  FX {K_s[0,0]:.2f}  FY {K_s[1,1]:.2f}  CX {K_s[0,2]:.2f}  CY {K_s[1,2]:.2f}')
    print('  DIST ' + ', '.join(f'{x:.5f}' for x in D_s))

    print('\nfisheye model (K + 4 coefficients):')
    try:
        rms_f, K_f, D_f, keep_f, err_f = solve_with_pruning(solve_fisheye, obj, img, names, True)
        print(f'  {len(keep_f)} frames, rms {rms_f:.3f} px')
        print(f'  FX {K_f[0,0]:.2f}  FY {K_f[1,1]:.2f}  CX {K_f[0,2]:.2f}  CY {K_f[1,2]:.2f}')
        print('  D ' + ', '.join(f'{x:.5f}' for x in D_f))
    except cv2.error as e:
        print(f'  failed: {e}')
        rms_f = K_f = D_f = None
        keep_f, err_f = [], {}

    print(f'\nborrowed (lane_camera.py now): FX {K_OLD[0,0]}  FY {K_OLD[1,1]}  CX {K_OLD[0,2]}  CY {K_OLD[1,2]}')

    res = {
        'folders': args.folders, 'board_inner_corners': size, 'square_m': args.square,
        'image_size': [IMG_W, IMG_H], 'frames_found': len(obj),
        'standard': {'rms_px': rms_s, 'K': K_s.tolist(), 'DIST': D_s.tolist(),
                     'frames_used': len(keep_s)},
    }
    if K_f is not None:
        res['fisheye'] = {'rms_px': rms_f, 'K': K_f.tolist(), 'D': D_f.tolist(),
                          'frames_used': len(keep_f)}
    with open(os.path.join(out, 'calib_result.json'), 'w') as fh:
        json.dump(res, fh, indent=2)
    with open(os.path.join(out, 'per_frame.csv'), 'w') as fh:
        fh.write('frame,err_standard_px,err_fisheye_px\n')
        for n in names:
            s = f'{err_s[n]:.3f}' if n in err_s else 'dropped'
            fz = f'{err_f[n]:.3f}' if n in err_f else ('dropped' if K_f is not None else '')
            fh.write(f'{n},{s},{fz}\n')

    # Side-by-side: raw | borrowed | new standard | new fisheye (2x2).
    idx = np.linspace(0, len(names) - 1, min(args.n_examples, len(names))).astype(int)
    for j in idx:
        raw = frames[names[j]]
        tiles = [label(raw, 'raw'),
                 label(undistort_std(raw, K_OLD, DIST_OLD), 'borrowed'),
                 label(undistort_std(raw, K_s, D_s), 'new standard')]
        tiles.append(label(undistort_fish(raw, K_f, D_f), 'new fisheye') if K_f is not None
                     else np.zeros_like(raw))
        grid_img = np.vstack([np.hstack(tiles[:2]), np.hstack(tiles[2:])])
        cv2.imwrite(os.path.join(out, f'undist_{os.path.splitext(names[j])[0]}.png'), grid_img)

    print(f'\nresults -> {out}')


if __name__ == '__main__':
    main()
