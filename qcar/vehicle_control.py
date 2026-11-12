# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --

#region : File Description and Imports

"""
vehicle_control.py

Skills acivity code for vehicle control lab guide.
Students will implement a vehicle speed and steering controller.
Please review Lab Guide - vehicle control PDF
"""
import signal
import socket
import numpy as np
from threading import Thread
import time
import random
from pal.products.qcar import QCar, QCarGPS, IS_PHYSICAL_QCAR

from pal.utilities.math import wrap_to_pi
from hal.content.qcar_functions import QCarEKF
from hal.products.mats import SDCSRoadMap
import argparse
from utils import YOLOReceiver, YOLODriveLogic

parser = argparse.ArgumentParser(prog='Vehicle control')
parser.add_argument('-c','--calibrate', default=False)
parser.add_argument('-n','--node_configuration', default=0)

args = parser.parse_args()
node_options = args.node_configuration

# choosing between two sets of nodes which avoids two way traffic
if node_options==0:
    valid_nodes=[10,2,4,14,20,22,9,7,0]
else:
    valid_nodes=[10,1,5,3,8,23,21,15,6]

# choosing random node sequence
# random.shuffle(valid_nodes)
# num_nodes=random.randint(6,10)
# nodeSequence=valid_nodes[:num_nodes]+[valid_nodes[0]]

# nodeSequence=[0,2,4,6,0]
nodeSequence=[0,2,4,14,16,18,12,0]

print(f"Selected nodes: {nodeSequence}")
#================ Experiment Configuration ================
# ===== Timing Parameters
# - tf: experiment duration in seconds.
# - startDelay: delay to give filters time to settle in seconds.
# - controllerUpdateRate: control update rate in Hz. Shouldn't exceed 500
tf = 6000
startDelay = 1
controllerUpdateRate = 200
startTime = time.time()
def elapsed_time():
    return time.time() - startTime

# ===== Speed Controller Parameters
# - v_ref: desired velocity in m/s
# - K_p: proportional gain for speed controller
# - K_i: integral gain for speed controller
# v_ref = 0.75
# v_ref = 0.4
v_ref = 0.2   # first camera-steering tests (2026-11-11); back to 0.4 once proven
# v_ref = 0
K_p = 0.1
K_i = 1

# ===== Steering Controller Parameters
# - enableSteeringControl: whether or not to enable steering control
# - K_stanley: K gain for stanley controller
# - nodeSequence: list of nodes from roadmap. Used for trajectory generation.
enableSteeringControl = True
# K_stanley = 0.7
K_stanley = 1.5

# - sterring_trim: offset added to the steering command to cancel the car's
# lefthand steering bias (rad). Negative = steer more right.set
steering_trim = -0.14

# - gps_scale: GPS x/y are divided by this to line them up with the roadmap.
#   1.035 fitted the old calibration; with the current one (walls + two
#   landmarks) readings are within +-5 cm without correction (check_pose round 8).
# gps_scale = 1.035
gps_scale = 1.0

# - gps_corrections: sideways GPS errors measured with check_pose.py (round 9,
#   current calibration, lower part of the map, which has no walls nearby).
#   Each row is (x, y, dx, dy): at raw GPS position (x, y), add (dx, dy).
#   Nearby rows are blended (gps_correction_radius, m); ~1 m from every row the
#   correction fades to zero, so the upper part of the map is unchanged.
#   Re-measure after any recalibration.
#   Node 0, mid 0 -> 2 and mid 2 -> 4 are tuned from driving, not the survey.
#   0 -> 2: round 9 values crossed the divider (drive 4), 6.1 cm away from
#   the divider put the car on the outer edge (drive 5), so 3 cm away.
#   2 -> 4: 7.3 cm still left the rear left tyre on the divider (drive 5),
#   so 9 cm.
gps_corrections = np.array([
    # x      y      dx      dy
    [0.06, -0.06,  0.030,  0.000],   # node 0 (round 9: -0.058, 0.000)
    [0.27, -0.81,  0.021,  0.021],   # mid 0 -> 2 (round 9: -0.013, -0.013)
    [1.26, -1.15,  0.000,  0.063],   # node 2
    [2.04, -0.88, -0.064,  0.064],   # mid 2 -> 4 (round 9: -0.051, 0.052)
    [2.32,  0.00, -0.061,  0.000],   # node 4
    [2.29,  1.43, -0.036,  0.000],   # mid 4 -> 14
])
gps_correction_radius = 0.35
# - use_gps_table: False = gps_corrections is ignored (camera lane correction
#   replaces it, 2026-11-11). The table is kept above as a fallback.
use_gps_table = False

def correct_gps(x, y):
    """Raw GPS x/y -> x/y lined up with the roadmap (gps_scale, then the
    position-dependent gps_corrections if use_gps_table)."""
    x, y = x / gps_scale, y / gps_scale
    if not use_gps_table:
        return x, y
    d2 =(gps_corrections[:, 0] - x)**2 + (gps_corrections[:, 1] - y)**2
    w = np.exp(-d2 / gps_correction_radius**2)
    # the 0.05 fades the correction out where no row is nearby
    dx, dy = w @ gps_corrections[:, 2:] / (w.sum() + 0.05)
    return x + dx, y + dy

# - GPS-jump check: a GPS reading further than gps_max_jump (m) or
#   gps_max_heading_jump (rad) from the EKF estimate is ignored. If no reading
#   is accepted for gps_timeout (s), the car is stopped.
gps_max_jump = 0.3
gps_max_heading_jump = np.radians(30)
gps_timeout = 2.0

# - Camera lane correction (plan step 5, 2026-11-11). lane_camera.py sends the
#   car's offset from the lane centre over UDP. While the camera is trusted,
#   cam_bias = camera offset - GPS offset (low-pass, cam_tau s) is learned; it
#   is held when the camera isn't trusted (curves, intersections). The point
#   Stanley steers from is shifted sideways by cam_bias, so on straights the
#   cross-track equals the camera's offset. Learned while parked during
#   startDelay too, so the car starts from the camera's view of the lane.
#   Trusted = mode 'both', conf >= cam_min_conf, younger than cam_max_age, and
#   (once driving) within cam_max_disagree of GPS + cam_bias. |cam_bias| is
#   capped at cam_max_bias. Start lane_camera.py BEFORE this script.
use_camera = True
cam_port = 18555
cam_tau = 0.3
cam_min_conf = 0.6
cam_max_age = 0.2
cam_max_disagree = 0.10
cam_max_bias = 0.15

# Define the calibration pose
# Calibration pose is either [0,0,-pi/2] or [0,2,-pi/2]
# Comment out the one that is not used 
#calibrationPose = [0,0,-np.pi/2]
calibrationPose = [0,2,-np.pi/2]
# calibrate = 'y' in input('do you want to recalibrate?(y/n)')
calibrate = args.calibrate 

#endregion
# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --

#region : Initial setup
def remove_backward_points(wp):
    """Drop waypoints that step backwards along the path. generate_path can
    overshoot a node by a few mm (e.g. node 16), so the next segment points
    backwards; SteeringController never gets past such a segment and drives
    off along its extension."""
    keep = [0, 1]
    for i in range(2, wp.shape[1]):
        a, b = wp[:, keep[-2]], wp[:, keep[-1]]
        if np.dot(wp[:, i] - b, b - a) > 0:
            keep.append(i)
    return wp[:, keep]

if enableSteeringControl:
    roadmap = SDCSRoadMap(leftHandTraffic=False)
    waypointSequence = remove_backward_points(roadmap.generate_path(nodeSequence))
    initialPose = roadmap.get_node_pose(nodeSequence[0]).squeeze()
else:
    initialPose = [0, 0, 0]


# Used to enable safe keyboard triggered shutdown
global KILL_THREAD
KILL_THREAD = False
def sig_handler(*args):
    global KILL_THREAD
    KILL_THREAD = True
signal.signal(signal.SIGINT, sig_handler)
#endregion

class SpeedController:

    def __init__(self, kp=0, ki=0):
        self.maxThrottle = 0.3

        self.kp = kp
        self.ki = ki

        self.ei = 0


    # ==============  SECTION A -  Speed Control  ====================
    def update(self, v, v_ref, dt):
        
        e = v_ref - v
        self.ei += dt*e

        return np.clip(
            self.kp*e + self.ki*self.ei,
            -self.maxThrottle,
            self.maxThrottle
        )
    
        return 0

class SteeringController:

    def __init__(self, waypoints, k=1, cyclic=True):
        self.maxSteeringAngle = np.pi/6

        self.wp = waypoints
        self.N = len(waypoints[0, :])
        self.wpi = 0

        self.k = k
        self.cyclic = cyclic

        self.p_ref = (0, 0)
        self.th_ref = 0

    # ==============  SECTION B -  Steering Control  ====================
    def update(self, p, th, speed):
        wp_1 = self.wp[:, np.mod(self.wpi, self.N-1)]
        wp_2 = self.wp[:, np.mod(self.wpi+1, self.N-1)]
        
        v = wp_2 - wp_1
        v_mag = np.linalg.norm(v)
        try:
            v_uv = v / v_mag
        except ZeroDivisionError:
            return 0

        tangent = np.arctan2(v_uv[1], v_uv[0])

        s = np.dot(p-wp_1, v_uv)

        if s >= v_mag:
            if  self.cyclic or self.wpi < self.N-2:
                self.wpi += 1

        ep = wp_1 + v_uv*s
        ct = ep - p
        dir = wrap_to_pi(np.arctan2(ct[1], ct[0]) - tangent)

        ect = np.linalg.norm(ct) * np.sign(dir)
        psi = wrap_to_pi(tangent-th)

        self.p_ref = ep
        self.th_ref = tangent

        # Print what the controller sees while it drices
        self.ect = ect

        return np.clip(
            wrap_to_pi(psi + np.arctan2(self.k*ect, speed)),
            -self.maxSteeringAngle,
            self.maxSteeringAngle)
        
        # return 0

class CameraReceiver:
    """Latest lane_camera.py result from UDP (non-blocking)."""

    def __init__(self, port):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('127.0.0.1', port))
        self.sock.setblocking(False)
        self.t, self.mode, self.offset, self.heading, self.conf = 0.0, 'none', np.nan, np.nan, 0.0
        self.count = 0

    def read(self):
        """Drain the socket and keep the newest message."""
        while True:
            try:
                data = self.sock.recv(256)
            except (BlockingIOError, socket.error):
                return
            try:
                t, mode, off, head, conf = data.decode().split(',')
                self.t, self.mode = float(t), mode
                self.offset = float(off) / 100       # cm -> m, + = left of lane centre
                self.heading = np.radians(float(head))
                self.conf = float(conf)
                self.count += 1
            except ValueError:
                pass

def path_left_offset(ctrl, p):
    """Sideways distance of point p from the controller's current path
    segment, + = left of the path (same sign as the camera offset).
    Also returns the left unit normal of the segment."""
    wp_1 = ctrl.wp[:, np.mod(ctrl.wpi, ctrl.N-1)]
    wp_2 = ctrl.wp[:, np.mod(ctrl.wpi+1, ctrl.N-1)]
    t = (wp_2 - wp_1) / np.linalg.norm(wp_2 - wp_1)
    n_left = np.array([-t[1], t[0]])
    return float(np.dot(p - wp_1, n_left)), n_left

def controlLoop():
    #region controlLoop setup
    global KILL_THREAD
    u = 0
    delta = 0
    x = 0.2
    y = 0.6
    th = 0.7

    #endregion

    #region Controller initialization
    speedController = SpeedController(
        kp=K_p,
        ki=K_i
    )
    if enableSteeringControl:
        steeringController = SteeringController(
            waypoints=waypointSequence,
            k=K_stanley
        )
    #endregion

    #region QCar interface setup
    qcar = QCar(readMode=1, frequency=controllerUpdateRate)
    yolo = YOLOReceiver()
    yoloDrive = YOLODriveLogic(pulseLength=controllerUpdateRate*3)

    if enableSteeringControl:
        ekf = QCarEKF(x_0=initialPose)
        gps = QCarGPS(initialPose=calibrationPose,calibrate=calibrate)
    else:
        gps = memoryview(b'')
    #endregion

    # check the initial pose of the car 
    new = False
    while not new:
        new = gps.readGPS()
    initPose = np.array([*correct_gps(gps.position[0], gps.position[1]),
                         gps.orientation[2]
                         ])
    # Start the EKF from where the car actually is, so the GPS-jump check
    # doesn't reject the first readings
    ekf = QCarEKF(x_0=initPose)
    startNodeReached, initWaypointSequence = roadmap.initial_check(initPose, nodeSequence,waypointSequence)
    if not startNodeReached:
        initWaypointSequence = remove_backward_points(initWaypointSequence)
        initSteeringController = SteeringController(waypoints=initWaypointSequence,
                                                    k=K_stanley,
                                                    cyclic=False)
    cam = CameraReceiver(cam_port) if (use_camera and enableSteeringControl) else None
    camBias = 0.0
    camStartReported = False
    with qcar, gps, yolo:
        if calibrate:
            return
        t0 = time.time()
        t=0
        startTime = time.time()
        lastGoodGPS = 0
        while (t < tf+startDelay) and (not KILL_THREAD):
            #region : Loop timing update
            # start = elapsed_time()
            tp = t
            t = time.time() - t0
            dt = t-tp
            #endregion

            #region : Read from sensors and update state estimates
            qcar.read()

            # point_A = elapsed_time()
            yolo.read()
            vGain = yoloDrive.check_yolo(yolo.stopSign,
                                         yolo.trafficlight,
                                         yolo.cars, 
                                         yolo.yieldSign,
                                         yolo.person)
            if enableSteeringControl:

                if gps.readGPS():
                    y_gps = np.array([
                        *correct_gps(gps.position[0], gps.position[1]),
                        gps.orientation[2]
                    ])
                    # GPS-jump check: ignore readings the car can't have moved to
                    jump = np.linalg.norm(y_gps[:2] - ekf.x_hat[:2,0])
                    headingJump = abs(wrap_to_pi(y_gps[2] - ekf.x_hat[2,0]))
                    if jump < gps_max_jump and headingJump < gps_max_heading_jump:
                        lastGoodGPS = t
                    else:
                        print(f"GPS rejected: ({y_gps[0]:.2f},{y_gps[1]:.2f},{np.degrees(y_gps[2]):.0f}deg)  jump={jump:.2f} m  heading_jump={np.degrees(headingJump):.0f}deg")
                        y_gps = None
                    ekf.update(
                        [qcar.motorTach, delta],
                        dt,
                        y_gps,
                        qcar.gyroscope[2],
                    )
                    if y_gps is not None:
                        print(f"GPS: ({y_gps[0]:.2f},{y_gps[1]:.2f},{np.degrees(y_gps[2]):.0f}deg)")
                else:
                    ekf.update(
                        [qcar.motorTach, delta],
                        dt,
                        None,
                        qcar.gyroscope[2],
                    )

                if t - lastGoodGPS > gps_timeout:
                    print(f"STOPPED: no GPS reading accepted for {gps_timeout:.0f} s. "
                          f"Last estimate ({ekf.x_hat[0,0]:.2f},{ekf.x_hat[1,0]:.2f},{np.degrees(ekf.x_hat[2,0]):.0f}deg)")
                    break

                x = ekf.x_hat[0,0]
                y = ekf.x_hat[1,0]
                th = ekf.x_hat[2,0]
                p = ( np.array([x, y])
                    + np.array([np.cos(th), np.sin(th)]) * 0.1)

                # Camera lane correction: learn cam_bias while the camera is
                # trusted, hold it otherwise, and steer from p shifted by it.
                p_steer = p
                camUsed = False
                if cam is not None:
                    cam.read()
                    ctrl = steeringController if startNodeReached else initSteeringController
                    gpsOff, nLeft = path_left_offset(ctrl, p)
                    camOk = (cam.mode == 'both' and cam.conf >= cam_min_conf
                             and time.time() - cam.t < cam_max_age)
                    if camOk and t >= startDelay:
                        # watchdog once driving; while parked the first
                        # (possibly large) bias must be allowed in
                        camOk = abs(cam.offset - (gpsOff + camBias)) < cam_max_disagree
                    if camOk:
                        a = 1 - np.exp(-dt / cam_tau)
                        camBias = float(np.clip(camBias + a*(cam.offset - gpsOff - camBias),
                                                -cam_max_bias, cam_max_bias))
                        camUsed = True
                    p_steer = p + nLeft * camBias
                    if not camStartReported and t >= startDelay:
                        camStartReported = True
                        if cam.count == 0:
                            print("CAMERA: no data from lane_camera.py, driving on GPS only (cam_bias = 0)")
                        else:
                            print(f"CAMERA: bias at start {camBias*100:+.1f} cm  "
                                  f"(camera {cam.offset*100:+.1f} cm, GPS {gpsOff*100:+.1f} cm, {cam.count} messages)")

            v = qcar.motorTach
            #endregion

            #region : Update controllers and write to car
            if t < startDelay:
                u = 0
                delta = 0
            else:
                #region : Speed controller update
                u = speedController.update(v, v_ref*vGain, dt)
                #endregion

                #region : Steering controller update
                if enableSteeringControl:
                    if not startNodeReached:
                        # check the criterion inside the if condition to ensure it will
                        # only be executed once
                        distToStart = np.linalg.norm(waypointSequence[:,0]-p)
                        startNodeReached = distToStart < 0.2
                        # if the start node is not reached, use the initial steering controller
                        delta = initSteeringController.update(p_steer, th, v)
                    else:
                        delta = steeringController.update(p_steer, th, v)
                        if int(t*2) != int((t-dt)*2):   # about twice per second
                            camTxt = ""
                            if cam is not None:
                                camTxt = (f"  cam={cam.mode}/{cam.offset*100:+.1f}cm  "
                                          f"bias={camBias*100:+.1f}cm  used={int(camUsed)}")
                            print(f"pose=({x:.2f},{y:.2f},{np.degrees(th):.0f}deg)  path_pt=({steeringController.p_ref[0]:.2f},{steeringController.p_ref[1]:.2f})  cross_track={steeringController.ect:.3f} m{camTxt}  t_epoch={time.time():.2f}")
                else:
                    delta = 0
                #endregion
            qcar.write(u, delta + steering_trim)            
            # end = elapsed_time()
            continue
        qcar.read_write_std(throttle= 0, steering= 0)

# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --

#region : Setup and run experiment
if __name__ == '__main__':
    
    #sleep for 10 seconds to wait for YOLO server to start
    time.sleep(10)
    
    #region : Setup control thread, then run experiment
    controlThread = Thread(target=controlLoop)
    controlThread.start()

    try:
        while controlThread.is_alive() and (not KILL_THREAD):
            # MultiScope.refreshAll()
            time.sleep(0.01)
    finally:
        KILL_THREAD = True
