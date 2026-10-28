"""
check_pose.py

Static localization check. Prints the live GPS pose and how far it is
sideways from the route path. Place the car by hand, centered in its lane,
at different spots along the route and compare: a centered car should read
an offset near 0. Motors are not used, so the car can be moved while it runs.

Run on the QCar:  python check_pose.py      (Ctrl+C to stop)
"""
import time
import numpy as np
from pal.products.qcar import QCarGPS
from hal.products.mats import SDCSRoadMap

# Must match vehicle_control.py
nodeSequence = [0, 2, 4, 14, 16, 18, 12, 0]
calibrationPose = [0, 2, -np.pi/2]

roadmap = SDCSRoadMap(leftHandTraffic=False)
path = roadmap.generate_path(nodeSequence)  # 2 x N waypoints

def path_offset(pos):
    """Signed sideways distance from the path (+ = left of path direction),
    the closest path point, and the path heading there."""
    i = int(np.argmin(np.linalg.norm(path.T - pos, axis=1)))
    i = min(i, path.shape[1] - 2)
    t = path[:, i+1] - path[:, i]
    t = t / np.linalg.norm(t)
    r = pos - path[:, i]
    return t[0]*r[1] - t[1]*r[0], path[:, i], np.arctan2(t[1], t[0])

# Readings jitter, so report statistics over the last N readings
# (about 4-5 s). Hold the car still until n reaches N before reading.
N = 25
offsets, headingErrs, positions = [], [], []

gps = QCarGPS(initialPose=calibrationPose, calibrate=False)
lastPrint = 0
try:
    with gps:
        while True:
            if gps.readGPS():
                pos = np.array([gps.position[0], gps.position[1]])
                th = gps.orientation[2]
                off, pt, tangent = path_offset(pos)
                hErr = np.arctan2(np.sin(th-tangent), np.cos(th-tangent))

                # Start a fresh window when the car is moved (> 10 cm jump)
                if positions and np.linalg.norm(pos - np.mean(positions, axis=0)) > 0.10:
                    offsets, headingErrs, positions = [], [], []
                offsets = (offsets + [off])[-N:]
                headingErrs = (headingErrs + [hErr])[-N:]
                positions = (positions + [pos])[-N:]

                if time.time() - lastPrint > 0.5:
                    lastPrint = time.time()
                    mean = np.mean(positions, axis=0)
                    print(f"n={len(offsets):2d}/{N}  "
                          f"GPS=({mean[0]:.2f},{mean[1]:.2f},{np.degrees(th):.0f}deg)  "
                          f"nearest_path=({pt[0]:.2f},{pt[1]:.2f},{np.degrees(tangent):.0f}deg)  "
                          f"side_offset={np.mean(offsets):+.3f} m "
                          f"(spread {np.min(offsets):+.3f}..{np.max(offsets):+.3f})  "
                          f"heading_err={np.degrees(np.mean(headingErrs)):+.1f}deg")
            time.sleep(0.01)
except KeyboardInterrupt:
    pass
