"""
csi_snapshot.py

Camera lane correction, step 0 ("look first"). Saves one frame from each of
the four CSI cameras, plus a copy with the yellow HSV mask from
examples/QCar2_task_lane_following_probe.py painted red, so we can see:
  - which way each camera faces and how much floor it sees,
  - whether the yellow divider and the white edge line are in view,
  - whether the example's yellow threshold picks up the divider.

Motors and GPS are not used. Park the car centred in its lane first.

Run on the QCar:  python csi_snapshot.py right_lane
                  python csi_snapshot.py left_lane
Output: snapshots/<label>_<camera>.png and <label>_<camera>_yellow.png
"""
import os
import sys
import time
import numpy as np
import cv2
from pal.products.qcar import QCarCameras

label = sys.argv[1] if len(sys.argv) > 1 else 'snap'

# 820x410 matches the fisheye calibration in skills_activities/02-image_interpretation
imageWidth, imageHeight, frameRate = 820, 410, 30

# Yellow bounds from the lane-following example (OpenCV HSV: H 0-180)
yellowLower = np.array([10, 50, 100])
yellowUpper = np.array([45, 255, 255])

outDir = 'snapshots'
os.makedirs(outDir, exist_ok=True)

cams = QCarCameras(frameWidth=imageWidth, frameHeight=imageHeight,
                   frameRate=frameRate, enableRight=True, enableBack=True,
                   enableLeft=True, enableFront=True)
names = ['right', 'back', 'left', 'front']
try:
    # Let auto exposure settle for ~2 s before keeping a frame
    t0 = time.time()
    while time.time() - t0 < 2.0:
        cams.readAll()
        time.sleep(1 / frameRate)

    for name in names:
        cam = getattr(cams, 'csi' + name.capitalize(), None)
        if cam is None or cam.imageData is None or not cam.imageData.any():
            print(f'{name:5s}: no image')
            continue
        img = cam.imageData.copy()
        mask = cv2.inRange(cv2.cvtColor(img, cv2.COLOR_BGR2HSV),
                           yellowLower, yellowUpper)
        overlay = img.copy()
        overlay[mask > 0] = (0, 0, 255)
        cv2.imwrite(os.path.join(outDir, f'{label}_{name}.png'), img)
        cv2.imwrite(os.path.join(outDir, f'{label}_{name}_yellow.png'), overlay)
        print(f'{name:5s}: saved, {100 * np.count_nonzero(mask) / mask.size:.1f} % yellow pixels')
finally:
    cams.terminate()
