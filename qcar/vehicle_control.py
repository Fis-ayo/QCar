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
from lane_fusion import axle_step, LaneFusion   # §57: axle dead reckoning; §62: the fusion filter

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
v_ref = 0.4   # camera steering at 0.4 m/s from 2026-11-18 (camera report §41)
# v_ref = 0.2   # first camera-steering drives (2026-11-11 ... 11-18, §36-§40)
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
#   2026-11-19 (camera report §47): -0.14 -> -0.04. Gyro + speed in the 11-18 and
#   11-19 drives: with -0.14 the controller had to command ~+6 deg to drive straight
#   (car held 1-2 cm right of centre) and only ~21 deg of left steering was reached at
#   the 30 deg limit (roundabout needs ~22). Straight-ahead point measured at
#   trim + 5.7 deg ~ -2.3 deg. REVERT: set back to -0.14 (tuned 2026-10-28 from GPS).
# steering_trim = -0.14
steering_trim = -0.04

# - K_soft: Stanley softening speed (m/s), cross-track term atan(K_stanley*e/(v+K_soft)).
#   Keeps the term finite when the car is (nearly) stopped, e.g. at the start
#   (2026-11-19, §47). REVERT: K_soft = 0.0 gives the old controller exactly.
K_soft = 0.05

# - Curvature speed limit (camera steering only, 2026-11-19, §47): the speed target is
#   min(v_ref, sqrt(v_curve_alat / k)) for the largest |map curvature| k from
#   cs_wheelbase behind to v_curve_ahead m ahead of the front axle (looking ahead is
#   for speed planning only, not steering), and never below v_curve_min. With 0.13
#   m/s^2: straights 0.4, 0 -> 2 / 2 -> 4 (k 1.12) 0.34, roundabout (k 1.48) 0.30,
#   16 -> 18 (k 1.61) 0.28 m/s. More camera frames per metre and less delay in the
#   tight curves. REVERT: use_curve_speed = False.
#   OFF for the next drive (user, 2026-11-19): test the new trim alone first, then
#   switch this on and compare the same curves (report "Where to pick up").
use_curve_speed = True
v_curve_alat = 0.13         # m/s^2
v_curve_min = 0.2           # m/s
v_curve_ahead = 0.5         # m

# - gps_scale: GPS x/y are divided by this to line them up with the roadmap.
#   1.035 fitted the old calibration; with the current one (walls + two
#   landmarks) readings are within +-5 cm without correction (check_pose round 8).
# gps_scale = 1.035
gps_scale = 1.0

# (The position-dependent gps_corrections table was removed on 2026-11-19, camera
#  report §43: the camera does the lane keeping, nothing is tuned per place.)

def correct_gps(x, y):
    """Raw GPS x/y -> x/y lined up with the roadmap (gps_scale only)."""
    return x / gps_scale, y / gps_scale

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
use_camera = False
cam_port = 18555
cam_tau = 0.3
cam_min_conf = 0.6
cam_max_age = 0.2
cam_max_disagree = 0.10
cam_max_bias = 0.15

# - Camera steering (plan step C3, camera report §35, 2026-11-18). When on, the
#   car steers from the camera's offset and heading to the lane, not from the
#   GPS sideways position (which was ~8 cm off at the start, §34):
#   * position along the route s (front axle) = start from the first GPS reading
#     (nearest route point whose direction matches the GPS heading, anywhere on
#     the route) + enc_scale * encoder distance, nudged towards the GPS position
#     along the route when a reading agrees within cs_gps_gate (gain cs_gps_gain).
#   * the map's lane curvature around s is sent to lane_camera.py (profile_port),
#     which fits the lane lines with that shape and returns offset + heading.
#   * between camera fits (and where it is blind, e.g. the start of 0 -> 2) the
#     offset and heading are carried on with the encoder and the gyro.
#   * Stanley steers from the front axle point placed at that offset from the
#     route, with that heading. On an arc its heading term then gives the turn.
#   * safety stops: no camera fit for cs_dr_max m, |offset| > cs_max_offset, or
#     no message from lane_camera.py for cs_cam_timeout s. The GPS timeout stop
#     is off in this mode (the GPS isn't needed to steer).
#   Start parked on the route, pointing along it (within cs_start_lat of it by
#   GPS), ideally where the camera sees the lane, and start lane_camera.py (with
#   UDP) BEFORE this script. use_camera must be False.
use_camera_steer = True
enc_scale = 0.98            # encoder over-reads distance by 2 % (§34)
gps_front_ahead = 0.11      # m, front axle ahead of the GPS/EKF point (§34)
cs_gps_gain = 0.2           # fraction of the along-route GPS difference applied per accepted reading
cs_gps_gate = 0.10          # m, larger along-route differences are ignored
cs_gps_lat_gate = 0.15      # m, GPS readings further than this from the route are ignored
cs_max_head = np.radians(8)     # camera heading used only within this of the expected heading
cs_head_rms = 0.10          # cm, ... and only from a fit with less rms than this
cs_dr_max = 2.3             # m driven without a camera fit -> stop (0 -> 2 is bridged for up to ~1.8 m since §40; was 1.8)
cs_max_offset = 0.12        # m, |offset| beyond this -> stop
cs_cam_timeout = 0.5        # s without a lane_camera.py message -> stop
cs_wheelbase = 0.256        # m
cs_start_lat = 0.15         # m, refuse to start if the first GPS reading is further than this from the route
cs_start_head = np.radians(90)  # route points pointing more than this away from the GPS heading aren't start candidates
# jump gate (§36): a camera offset further than cs_gate0 + cs_gate_per_m * (m since
# the last accepted fit) from the predicted one is ignored, unless the camera keeps
# saying the same (sd < cs_relock_sd) for cs_relock_dist m. Catches single bad fits
# (edge search on the divider near node 14: -27 cm; intersection edge at node 2: +9 cm).
cs_gate0 = 0.04             # m
cs_gate_per_m = 0.03        # m per m of dead reckoning
cs_relock_dist = 0.15       # m
cs_relock_sd = 0.02         # m
# confirmation (§36): after cs_confirm_after m without an accepted fit, a new fit is
# used only when the next fit agrees with it within cs_confirm_tol (an isolated
# edge fit in the 0 -> 2 curve was 6 cm wrong and steered the car to the divider).
cs_confirm_after = 0.30     # m
cs_confirm_tol = 0.02       # m
# re-lock faster (§37): cs_relock_n consecutive ignored fits within cs_relock_n_sd
# of each other are accepted too (the gate had locked onto a drifted estimate).
cs_relock_n = 3
cs_relock_n_sd = 0.015      # m
# no camera heading near curve starts/ends (§37): if the map curvature changes by
# more than cs_head_kchange between cs_head_window m behind and ahead of the front
# axle, the fit's heading isn't used (offset/heading are ambiguous there, §24); the
# gyro carries the heading. A -25 deg heading at the 0 -> 2 exit made the car steer
# left onto the divider at node 2.
# whole fit rejected (offset too) if its heading is further than this from the
# expected heading (§40): near the 0 -> 2 exit / node 2 approach, edge fits read
# +9.5 ... +2 cm with -44 ... -35 deg (expected -16); acting on them pushed the car
# to the outer edge. Real fits in all drives so far were within 13 deg.
cs_max_head_fit = np.radians(15)
cs_head_window = (-0.30, 0.60)
cs_head_kchange = 0.3       # 1/m
# In camera steering, Stanley's heading error uses the exact lane tangent at the front
# axle (route.heading(s)), not the waypoint segment's direction (§45). (Curvature
# feedforward was tried and removed, §44-§46: front-axle Stanley already turns in at
# the rate the body rotates.)
profile_port = 18556
profile_s = np.round(np.arange(-0.30, 1.0001, 0.05), 3)   # keep identical to lane_camera.PROFILE_S
profile_period = 0.02       # s between curvature messages
# Dead reckoning and along-route correction (camera report §53-§57, 2026-11-20). The
# ruler check in the roundabout (§55: front axle 11 cm, rear 3 cm outward while the
# estimate said +2 cm) is reproduced offline only with both (§56):
# - cs_dr_model 'axle': the rear axle is carried along the body heading (encoder + gyro),
#   the front axle is rear + wheelbase. The old model moved the rear axle's offset but
#   used it as the front axle's, so on arcs the car steered the rear axle onto the lane
#   and the front ran 4-5 cm outward (§53). The body heading psi is the state, so an
#   along-route correction (GPS, camera) changes the heading to the lane, not the gyro
#   heading. REVERT: cs_dr_model = 'old'.
# - cs_shift_use: lane_camera.py refits the lane shape with the route shifted along s
#   where the curvature changes in view (curve starts/ends) and sends the best shift;
#   two consecutive frames agreeing within cs_shift_confirm_tol move s to the camera's
#   value. In every drive the painted roundabout started curving 14-30 cm before the
#   estimate expected (late turn-in, §55). REVERT: cs_shift_use = False (or run
#   lane_camera.py with --no-shift).
cs_dr_model = 'axle'
cs_enc_axle = 'front'       # which axle's travel the scaled encoder distance is (§54)
cs_shift_use = True
cs_shift_gain = 1.0         # fraction of the camera's along-route difference applied
cs_shift_confirm_tol = 0.06 # m, the two frames' along-route positions must agree within this
# Since 2026-11-20 (§68): one usable frame (lane_camera's sh_ok: clean best fit, clear rms
# gain, inside +-0.30 m) is enough. In drive D (exit, +14 cm) and car B (entry, +24 cm) the
# second frame never came (no lane in view / rate-limited shift fit), nothing was applied and
# both crossed the divider at the 16 -> 18 S-bend; every offline replay (§56) used single
# frames, with no wrong correction on 10 drives. REVERT: cs_shift_confirm = True.
cs_shift_confirm = False
cs_shift_hist = 1.0         # s of (time, s) history kept to evaluate s at the frame time
# Estimator (camera report §61-§62, 2026-11-20): 'fusion' = lane_fusion.LaneFusion, a
# Kalman filter on (s, d, body heading, GPS bias along/across): prediction by encoder +
# gyro (cs_dr_model), updates by the camera fits (offset, heading where allowed), the
# camera along-route measurements at curve starts/ends (lane_camera sh_*) and EVERY GPS
# reading (x, y, heading, with place-dependent bias states; each reading is tested
# against the estimate and its uncertainty, instead of the jump gate / confirmation /
# re-lock rules). Offline on 8 drives it halved the error at camera re-acquisitions
# (8.3 -> 4.0 cm) and rejected no real camera fit (§61). The safety stops stay.
# REVERT: cs_estimator = 'c3' (the C3 estimator with the §57 additions).
# 2026-11-20 (§64): back to 'c3' after drive F stopped in the roundabout on the filter (§63).
# §72: 'fusion' again: with camera heading sd 6 deg, the live single-frame along-route
# measurements and the GPS sideways, the filter passes 11 of 12 recorded drives
# (replay_check.py), including car B's S-bend and blind 12 -> 0 straight and drive F.
# Speed and stop follow its uncertainty (cs_fus_*). REVERT: cs_estimator = 'c3' (drive G's setup).
# §74 (2026-11-20, end of day): back to 'c3' after drive H stopped in the roundabout on the
# filter (§73); the filter's roundabout behaviour is to be investigated offline first.
cs_estimator = 'c3'
cs_fus_shift_sd = (0.04, 0.07)   # m: sd of a camera along-route measurement, fit rms < 0.06 / otherwise
# Fusion filter only (§72): speed and stop from the filter's offset uncertainty sd_d. On the
# recorded drives sd_d is ~0.5 cm where the camera sees the lane, 3.5-3.8 cm on the blind part of
# 0 -> 2, 3.6-5.0 cm in the roundabout's second half and the S-bend, ~2.2 cm on 12 -> 0 (GPS).
# Speed: v_ref below cs_fus_v_sd[0], linearly down to cs_fus_v_min at cs_fus_v_sd[1] (on top of
# the curvature limit). Stop when sd_d > cs_fus_sd_stop; the blind-distance backstop is longer
# than cs_dr_max (2.3 m) because the S-bend is blind for ~2.1 m.
cs_fus_v_sd = (0.015, 0.04)  # m
cs_fus_v_min = 0.2           # m/s
cs_fus_sd_stop = 0.07        # m
cs_fus_dr_max = 3.5          # m

# - Odometry log (camera report §32, 2026-11-18): logging only, no effect on
#   driving. Every odo_log_period s one row to odo_log_file with the same
#   columns as odometry_log.py's odometry.csv (so c2_offline_test.py reads it),
#   plus the EKF pose and the steering command. The gyro bias is averaged while
#   parked during startDelay; gyro_z_raw is logged too, to redo it offline.
log_odometry = True
odo_log_file = 'odometry_drive.csv'
odo_log_period = 0.02

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

class RoutePath:
    """The route as a function of the distance s along it (m, 0 = first node,
    cyclic: s is taken modulo the route length L, also for s < 0). Used by the
    camera steering (C3)."""

    def __init__(self, wp, step=0.01):
        xy = wp.T
        keep = np.r_[True, np.hypot(*np.diff(xy, axis=0).T) > 1e-6]
        xy = xy[keep]
        sw = np.r_[0, np.cumsum(np.hypot(*np.diff(xy, axis=0).T))]
        self.L = sw[-1]
        self.s = np.arange(0, self.L, step)
        self.x = np.interp(self.s, sw, xy[:, 0])
        self.y = np.interp(self.s, sw, xy[:, 1])
        self.th = np.unwrap(np.arctan2(np.gradient(self.y), np.gradient(self.x)))
        self.k = np.gradient(self.th, self.s)

    def xy(self, s):
        sm = np.mod(np.asarray(s, float), self.L)
        return np.stack([np.interp(sm, self.s, self.x), np.interp(sm, self.s, self.y)], -1)

    def heading(self, s):
        return np.interp(np.mod(np.asarray(s, float), self.L), self.s, self.th)

    def curvature(self, s):
        return np.interp(np.mod(np.asarray(s, float), self.L), self.s, self.k)

    def locate(self, x, y, th, max_head):
        """Start position: nearest route point to (x, y) anywhere on the route,
        among the points whose direction is within max_head of heading th (so a
        crossing road at an intersection isn't picked). -> (s, sideways
        distance, + = left of the route); falls back to the nearest point."""
        d2 = (self.x - x) ** 2 + (self.y - y) ** 2
        dh = np.abs(np.arctan2(np.sin(self.th - th), np.cos(self.th - th)))
        ok = dh < max_head
        i = int(np.argmin(np.where(ok, d2, np.inf))) if np.any(ok) else int(np.argmin(d2))
        lat = -np.sin(self.th[i]) * (x - self.x[i]) + np.cos(self.th[i]) * (y - self.y[i])
        return float(self.s[i]), float(lat)

    def project(self, x, y, s_guess, win):
        """Position along the route of point (x, y), searched within +-win of
        s_guess. -> (s, sideways distance, + = left of the route)."""
        cand = np.arange(s_guess - win, s_guess + win, 0.005)
        p = self.xy(cand)
        i = int(np.argmin((p[:, 0] - x) ** 2 + (p[:, 1] - y) ** 2))
        th = float(self.heading(cand[i]))
        lat = -np.sin(th) * (x - p[i, 0]) + np.cos(th) * (y - p[i, 1])
        return float(cand[i]), float(lat)


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

    def __init__(self, waypoints, k=1, cyclic=True, k_soft=0.0):
        self.maxSteeringAngle = np.pi/6

        self.wp = waypoints
        self.N = len(waypoints[0, :])
        self.wpi = 0

        self.k = k
        self.k_soft = k_soft
        self.cyclic = cyclic

        self.p_ref = (0, 0)
        self.th_ref = 0

    # ==============  SECTION B -  Steering Control  ====================
    def update(self, p, th, speed, tangent_ref=None):
        """Stanley: heading error + atan(k * cross-track / (speed + k_soft)).
        tangent_ref: lane direction for the heading error (rad); default = the
        current waypoint segment's direction (camera report §45)."""
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
        psi = wrap_to_pi((tangent if tangent_ref is None else tangent_ref) - th)

        self.p_ref = ep
        self.th_ref = tangent

        # Print what the controller sees while it drices
        self.ect = ect

        return np.clip(
            wrap_to_pi(psi + np.arctan2(self.k*ect, speed + self.k_soft)),
            -self.maxSteeringAngle,
            self.maxSteeringAngle)
        
        # return 0

class CameraReceiver:
    """Latest lane_camera.py result from UDP (non-blocking). Since 2026-11-18 the
    message also carries the map-shape fit (mf_*, camera steering C3)."""

    def __init__(self, port):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('127.0.0.1', port))
        self.sock.setblocking(False)
        self.t, self.mode, self.offset, self.heading, self.conf = 0.0, 'none', np.nan, np.nan, 0.0
        self.mf_src, self.mf_d, self.mf_h, self.mf_rms, self.mf_s = 'none', np.nan, np.nan, np.nan, np.nan
        self.sh_m, self.sh_ok, self.sh_rms = np.nan, False, np.nan   # along-route shift fit (§57, §62)
        self.count = 0
        self.tRecv = 0.0      # local time of the newest message

    def read(self):
        """Drain the socket and keep the newest message. -> True if a new one arrived."""
        new = False
        while True:
            try:
                data = self.sock.recv(512)
            except (BlockingIOError, socket.error):
                return new
            try:
                f = data.decode().split(',')
                t, mode, off, head, conf = f[:5]
                self.t, self.mode = float(t), mode
                self.offset = float(off) / 100       # cm -> m, + = left of lane centre
                self.heading = np.radians(float(head))
                self.conf = float(conf)
                if len(f) >= 10:
                    self.mf_src = f[5]
                    self.mf_d = float(f[6]) / 100    # cm -> m
                    self.mf_h = np.radians(float(f[7]))
                    self.mf_rms = float(f[8])        # cm
                    self.mf_s = float(f[9])
                if len(f) >= 12:
                    self.sh_m = float(f[10])
                    self.sh_ok = f[11].strip() == '1'
                    self.sh_rms = float(f[12]) if len(f) >= 13 else np.nan
                else:
                    self.sh_m, self.sh_ok, self.sh_rms = np.nan, False, np.nan
                self.count += 1
                self.tRecv = time.time()
                new = True
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
            k=K_stanley,
            k_soft=K_soft
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
                                                    cyclic=False,
                                                    k_soft=K_soft)
    cam = CameraReceiver(cam_port) if (use_camera and enableSteeringControl) else None
    camBias = 0.0
    camStartReported = False
    odoFile = None

    # Camera steering (C3) setup
    cs = None
    csAbort = None
    if use_camera_steer and enableSteeringControl:
        route = RoutePath(waypointSequence)
        sGps0, lat0 = route.locate(initPose[0], initPose[1], initPose[2], cs_start_head)
        s0 = sGps0 + gps_front_ahead
        print(f"CAMSTEER: start at s = {s0:+.2f} m along the route (front axle, route length "
              f"{route.L:.2f} m), {lat0*100:+.1f} cm from it by GPS")
        if use_camera:
            csAbort = "use_camera and use_camera_steer are both on; turn use_camera off"
        elif abs(lat0) > cs_start_lat:
            csAbort = (f"start position {lat0*100:+.1f} cm from the route by GPS "
                       f"(need within {cs_start_lat*100:.0f} cm, pointing along it)")
        camRx = CameraReceiver(cam_port)
        profSock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        cs = dict(s=s0, d=0.0, h=0.0, enc=None, biasSum=0.0, biasN=0, bias=None,
                  fitS=s0, lastProf=-1.0, hExpPrev=0.0, src='none', nFit=0, nGps=0,
                  rejS=None, rejVals=[], nRej=0, pend=None, nWait=0, nHbad=0,
                  psi=None, sHist=[], shPend=None, nShift=0, nGpsRej=0, gLast=None, gLastT=0.0)
        fus = None
        if cs_estimator == 'fusion':
            fus = LaneFusion(route, cs_wheelbase, gps_front_ahead, s0, initPose[2],
                             dr_model=cs_dr_model, enc_axle=cs_enc_axle)
            print(f"CAMSTEER: estimator = fusion filter (dead reckoning {cs_dr_model}, encoder = {cs_enc_axle} axle)")
        startNodeReached = True      # main controller from the start (the car starts on the route)
        # Stanley's waypoint index starts at 0 (node 0); move it to the segment the car is on
        wpS = np.r_[0, np.cumsum(np.hypot(*np.diff(steeringController.wp, axis=1)))]
        steeringController.wpi = int(np.clip(np.searchsorted(wpS, np.mod(s0, route.L), 'right') - 1,
                                             0, steeringController.N - 2))

    with qcar, gps, yolo:
        if calibrate:
            return
        if csAbort:
            print(f"CAMSTEER: not starting: {csAbort}")
            return
        if cs is not None:
            csCps = getattr(qcar, 'CPS_TO_MPS', None)
            if csCps is None:
                print("CAMSTEER: not starting: qcar.CPS_TO_MPS not available")
                return
        if log_odometry:
            odoFile = open(odo_log_file, 'w')
            odoFile.write('t_epoch,encoder,dist_m,tach_mps,dist_tach_m,gyro_z,heading_deg,'
                          'gyro_z_raw,x,y,th_deg,delta,cs_s,cs_d_cm,cs_h_deg,cs_src\n')
            cpsToMps = getattr(qcar, 'CPS_TO_MPS', np.nan)
            odoEnc0 = None
            odoBiasSum, odoBiasN, odoBias = 0.0, 0, None
            odoHeading = odoDistTach = 0.0
            odoLast = 0.0
            print(f"ODOMETRY: logging to {odo_log_file} (CPS_TO_MPS = {cpsToMps})")
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
            gpsRaw = None
            vTarget = v_ref                 # speed target (lowered in curves by the curvature speed limit)
            if enableSteeringControl:

                if gps.readGPS():
                    y_gps = np.array([
                        *correct_gps(gps.position[0], gps.position[1]),
                        gps.orientation[2]
                    ])
                    gpsRaw = y_gps.copy()     # before the jump check (camera steering has its own gate)
                    # GPS-jump check: ignore readings the car can't have moved to
                    jump = np.linalg.norm(y_gps[:2] - ekf.x_hat[:2,0])
                    headingJump = abs(wrap_to_pi(y_gps[2] - ekf.x_hat[2,0]))
                    if jump < gps_max_jump and headingJump < gps_max_heading_jump:
                        lastGoodGPS = t
                    else:
                        print(f"GPS rejected: ({y_gps[0]:.2f},{y_gps[1]:.2f},{np.degrees(y_gps[2]):.0f}deg)  jump={jump:.2f} m  heading_jump={np.degrees(headingJump):.0f}deg  t_epoch={time.time():.2f}")
                        y_gps = None
                    ekf.update(
                        [qcar.motorTach, delta],
                        dt,
                        y_gps,
                        qcar.gyroscope[2],
                    )
                    if y_gps is not None:
                        print(f"GPS: ({y_gps[0]:.2f},{y_gps[1]:.2f},{np.degrees(y_gps[2]):.0f}deg)  t_epoch={time.time():.2f}")
                else:
                    ekf.update(
                        [qcar.motorTach, delta],
                        dt,
                        None,
                        qcar.gyroscope[2],
                    )

                if cs is None and t - lastGoodGPS > gps_timeout:
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

                # Camera steering (C3): position along the route, offset and
                # heading to the lane, and the point Stanley steers from.
                thSteer = th
                tanRef = None
                if cs is not None:
                    newCam = camRx.read()
                    enc = int(qcar.motorEncoder[0])
                    gzRaw = float(qcar.gyroscope[2])
                    if cs['enc'] is None:
                        cs['enc'] = enc
                    if t < startDelay:
                        cs['biasSum'] += gzRaw
                        cs['biasN'] += 1
                    elif cs['bias'] is None:
                        cs['bias'] = cs['biasSum'] / max(cs['biasN'], 1)
                    # encoder: along the route; offset/heading carried on (dead reckoning)
                    ds = enc_scale * (enc - cs['enc']) * csCps
                    cs['enc'] = enc
                    if cs_estimator == 'fusion':
                        # §62: Kalman filter (lane_fusion.LaneFusion)
                        dpsi = (gzRaw - cs['bias']) * dt if cs['bias'] is not None else 0.0
                        fus.predict(ds, dpsi)
                        if gpsRaw is not None:
                            # every reading; the log repeats some within ~50 ms: use one
                            gKey = (round(float(gpsRaw[0]), 4), round(float(gpsRaw[1]), 4), round(float(gpsRaw[2]), 4))
                            if not (gKey == cs['gLast'] and time.time() - cs['gLastT'] < 0.1):
                                okG, _ = fus.gps(gpsRaw[0], gpsRaw[1], gpsRaw[2], 'full')
                                cs['nGps' if okG else 'nGpsRej'] += 1
                            cs['gLast'], cs['gLastT'] = gKey, time.time()
                        nowT = time.time()
                        cs['sHist'].append((nowT, float(fus.x[0])))
                        while cs['sHist'] and nowT - cs['sHist'][0][0] > cs_shift_hist:
                            cs['sHist'].pop(0)
                        if newCam:
                            status = 'DR'
                            # along-route measurement where the curvature changes in view
                            if (cs_shift_use and camRx.sh_ok and t >= startDelay and ds > 0
                                    and np.isfinite(camRx.mf_s) and np.isfinite(camRx.sh_m)):
                                sAtFrame = float(np.interp(camRx.t, [h_[0] for h_ in cs['sHist']],
                                                           [h_[1] for h_ in cs['sHist']]))
                                sTrue = camRx.mf_s + camRx.sh_m + (float(fus.x[0]) - sAtFrame)
                                sdS = cs_fus_shift_sd[0] if camRx.sh_rms < 0.06 else cs_fus_shift_sd[1]
                                sBefore = float(fus.x[0])
                                okS, _ = fus.camera_s(sTrue, sdS)
                                if okS:
                                    cs['nShift'] += 1
                                    status = 'shift'
                                    print(f"CAMSTEER: camera along-route measurement {100*(sTrue - sBefore):+.1f} cm -> "
                                          f"s {sBefore:.2f} -> {float(fus.x[0]):.2f} m  t_epoch={time.time():.2f}")
                            # lane fit: offset, and heading where the C3 rules allow it
                            if camRx.mf_src in ('YE', 'E', 'Y'):
                                hExpF = fus.h_exp()
                                if abs(wrap_to_pi(camRx.mf_h - hExpF)) > cs_max_head_fit:
                                    status = 'hbad'
                                    cs['nHbad'] += 1
                                else:
                                    kWin = route.curvature(fus.x[0] + np.linspace(cs_head_window[0], cs_head_window[1], 37))
                                    useH = (np.ptp(kWin) <= cs_head_kchange and camRx.mf_rms < cs_head_rms
                                            and abs(wrap_to_pi(camRx.mf_h - hExpF)) < cs_max_head)
                                    okC, _ = fus.camera(camRx.mf_src, camRx.mf_d, camRx.mf_h if useH else None,
                                                        camRx.mf_rms / 100)
                                    if okC:
                                        cs['fitS'] = float(fus.x[0])
                                        cs['nFit'] += 1
                                        status = camRx.mf_src
                                    else:
                                        cs['nRej'] += 1
                                        status = 'rej'
                            cs['src'] = status
                        cs['s'], cs['d'] = float(fus.x[0]), float(fus.x[1])
                        cs['h'] = fus.h()
                        thPath = float(route.heading(cs['s']))
                        hExp = fus.h_exp()
                    else:
                        if cs_dr_model == 'old':
                            sPrev = cs['s']
                            cs['s'] += ds
                            thPath = float(route.heading(cs['s']))
                            hExp = float(wrap_to_pi(float(route.heading(cs['s'] - cs_wheelbase)) - thPath))
                            cs['d'] += np.sin(cs['h'] - cs['hExpPrev']) * ds
                            if cs['bias'] is not None:
                                cs['h'] = float(wrap_to_pi(cs['h'] + (gzRaw - cs['bias']) * dt
                                                           - wrap_to_pi(thPath - float(route.heading(sPrev)))))
                            cs['hExpPrev'] = hExp
                        else:
                            # §57: rear axle carried along the body heading, front axle = rear + L
                            dpsi = (gzRaw - cs['bias']) * dt if cs['bias'] is not None else 0.0
                            if cs['psi'] is None:
                                cs['psi'] = cs['h'] + float(route.heading(cs['s']))
                            cs['s'], cs['d'], cs['psi'] = axle_step(route, cs_wheelbase, cs['s'], cs['d'],
                                                                    cs['psi'], ds, dpsi, cs_enc_axle)
                        # GPS: position along the route only, when it agrees
                        if gpsRaw is not None:
                            sG, latG = route.project(gpsRaw[0], gpsRaw[1], cs['s'] - gps_front_ahead, 0.4)
                            innov = sG + gps_front_ahead - cs['s']
                            if abs(innov) < cs_gps_gate and abs(latG) < cs_gps_lat_gate:
                                cs['s'] += cs_gps_gain * innov
                                cs['nGps'] += 1
                        if cs_dr_model != 'old':
                            # heading to the lane from the body heading at the (corrected) s
                            thPath = float(route.heading(cs['s']))
                            cs['h'] = float(wrap_to_pi(cs['psi'] - thPath))
                            hExp = float(wrap_to_pi(float(route.heading(cs['s'] - cs_wheelbase)) - thPath))
                        # (time, s) history: the camera's along-route shift is for its frame time
                        nowT = time.time()
                        cs['sHist'].append((nowT, cs['s']))
                        while cs['sHist'] and nowT - cs['sHist'][0][0] > cs_shift_hist:
                            cs['sHist'].pop(0)
                        # camera along-route correction at curve starts / ends (§55-§57)
                        if (newCam and cs_shift_use and camRx.sh_ok and t >= startDelay and ds > 0
                                and np.isfinite(camRx.mf_s) and np.isfinite(camRx.sh_m)):
                            hT = [h_[0] for h_ in cs['sHist']]
                            hS = [h_[1] for h_ in cs['sHist']]
                            dsCam = camRx.mf_s + camRx.sh_m - float(np.interp(camRx.t, hT, hS))
                            pend = cs['shPend']
                            confirmed = (pend is not None and cs['s'] - pend[1] < 0.3
                                         and abs(dsCam - pend[0]) < cs_shift_confirm_tol)
                            if confirmed or not cs_shift_confirm:
                                corr = cs_shift_gain * (0.5 * (dsCam + pend[0]) if confirmed else dsCam)
                                psiNow = cs['h'] + thPath            # keep the gyro heading
                                cs['s'] += corr
                                cs['sHist'] = [(a_, b_ + corr) for a_, b_ in cs['sHist']]   # no double correction
                                thPath = float(route.heading(cs['s']))
                                cs['h'] = float(wrap_to_pi(psiNow - thPath))
                                if cs_dr_model != 'old':
                                    cs['psi'] = psiNow
                                hExp = float(wrap_to_pi(float(route.heading(cs['s'] - cs_wheelbase)) - thPath))
                                cs['nShift'] += 1
                                cs['shPend'] = None
                                newCam = False                       # this frame's fit was made at the old s
                                cs['src'] = 'shift'
                                frTxt = (f"frames {pend[0]*100:+.1f} / {dsCam*100:+.1f} cm" if confirmed
                                         else f"single frame {dsCam*100:+.1f} cm")
                                print(f"CAMSTEER: along-route correction {corr*100:+.1f} cm from the camera "
                                      f"({frTxt}) -> s = {cs['s']:.2f} m  t_epoch={time.time():.2f}")
                            else:
                                cs['shPend'] = (dsCam, cs['s'])
                        # camera: offset (and heading when clean and plausible)
                        if newCam:
                            accept = False
                            status = 'DR'
                            headBad = (camRx.mf_src in ('YE', 'E', 'Y')
                                       and abs(wrap_to_pi(camRx.mf_h - hExp)) > cs_max_head_fit)
                            if camRx.mf_src not in ('YE', 'E', 'Y') or headBad:
                                cs['pend'] = None            # a frame without a usable fit breaks a confirmation pair
                                if headBad:
                                    status = 'hbad'
                                    cs['nHbad'] += 1
                            elif (t >= startDelay and cs['s'] - cs['fitS'] > cs_confirm_after
                                  and (cs['pend'] is None or abs(camRx.mf_d - cs['pend']) > cs_confirm_tol)):
                                # after a gap without fits: wait for a second fit that agrees (§36)
                                cs['pend'] = camRx.mf_d
                                cs['nWait'] += 1
                                status = 'wait'
                            else:
                                cs['pend'] = None
                                gate = cs_gate0 + cs_gate_per_m * (cs['s'] - cs['fitS'])
                                if t < startDelay or abs(camRx.mf_d - cs['d']) <= gate:
                                    accept = True
                                else:
                                    # jump gate: ignore, unless the camera keeps saying the same
                                    if cs['rejS'] is None:
                                        cs['rejS'], cs['rejVals'] = cs['s'], []
                                    cs['rejVals'].append(camRx.mf_d)
                                    cs['nRej'] += 1
                                    if ((cs['s'] - cs['rejS'] > cs_relock_dist
                                            and np.std(cs['rejVals'][-5:]) < cs_relock_sd)
                                            or (len(cs['rejVals']) >= cs_relock_n
                                                and np.std(cs['rejVals'][-cs_relock_n:]) < cs_relock_n_sd)):
                                        accept = True
                                        print(f"CAMSTEER: re-lock to the camera at s = {cs['s']:.2f} m: offset "
                                              f"{cs['d']*100:+.1f} -> {camRx.mf_d*100:+.1f} cm  t_epoch={time.time():.2f}")
                                    else:
                                        status = 'rej'
                                        if cs['nRej'] <= 50:
                                            print(f"CAMSTEER: camera offset {camRx.mf_d*100:+.1f} cm ignored "
                                                  f"(predicted {cs['d']*100:+.1f}, gate {gate*100:.1f} cm, "
                                                  f"{camRx.mf_src}) s = {cs['s']:.2f} m  t_epoch={time.time():.2f}")
                            if accept:
                                cs['rejS'] = None
                                cs['d'] = camRx.mf_d
                                kWin = route.curvature(cs['s'] + np.linspace(cs_head_window[0], cs_head_window[1], 37))
                                if (np.ptp(kWin) <= cs_head_kchange and camRx.mf_rms < cs_head_rms
                                        and abs(wrap_to_pi(camRx.mf_h - hExp)) < cs_max_head):
                                    cs['h'] = camRx.mf_h
                                    cs['psi'] = None             # body heading follows from the camera heading
                                cs['fitS'], cs['src'] = cs['s'], camRx.mf_src
                                cs['nFit'] += 1
                            elif t < startDelay and camRx.mode == 'both':
                                cs['d'], cs['h'] = camRx.offset, camRx.heading
                                cs['psi'] = None
                                cs['fitS'], cs['src'] = cs['s'], 'both'
                            else:
                                cs['src'] = status
                    # lane curvature around the car -> lane_camera.py
                    if t - cs['lastProf'] >= profile_period:
                        cs['lastProf'] = t
                        kk = route.curvature(cs['s'] + profile_s)
                        msg = f"{time.time():.3f},{cs['s']:.4f}," + ','.join(f"{v:.4f}" for v in kk)
                        profSock.sendto(msg.encode(), ('127.0.0.1', profile_port))
                    # Stanley steers from the front axle point at the offset, with the heading
                    nL = np.array([-np.sin(thPath), np.cos(thPath)])
                    p_steer = route.xy(cs['s']) + nL * cs['d']
                    tanRef = thPath           # heading error from the exact lane tangent at the front axle
                    thSteer = thPath + cs['h']
                    # curvature speed limit (speed planning: looks ahead, steering doesn't)
                    if use_curve_speed:
                        kMax = float(np.max(np.abs(route.curvature(
                            cs['s'] + np.linspace(-cs_wheelbase, v_curve_ahead, 40)))))
                        if kMax > 1e-6:
                            vTarget = min(v_ref, max(v_curve_min, float(np.sqrt(v_curve_alat / kMax))))
                    # fusion filter: slower when the offset estimate is uncertain (blind stretches, §72)
                    if fus is not None:
                        sdD = float(fus.sd()[1])
                        fU = float(np.clip((sdD - cs_fus_v_sd[0]) / (cs_fus_v_sd[1] - cs_fus_v_sd[0]), 0.0, 1.0))
                        vTarget = min(vTarget, v_ref - fU * (v_ref - cs_fus_v_min))
                    # safety stops (once driving)
                    stopWhy = None
                    if t >= startDelay:
                        if time.time() - camRx.tRecv > cs_cam_timeout:
                            stopWhy = (f"no message from lane_camera.py for {cs_cam_timeout:.1f} s "
                                       f"({camRx.count} received; is it running with UDP on?)")
                        elif cs['s'] - cs['fitS'] > (cs_fus_dr_max if fus is not None else cs_dr_max):
                            stopWhy = f"no camera fit for {cs['s'] - cs['fitS']:.2f} m"
                        elif fus is not None and float(fus.sd()[1]) > cs_fus_sd_stop:
                            stopWhy = f"offset uncertainty {100*float(fus.sd()[1]):.1f} cm"
                        elif abs(cs['d']) > cs_max_offset:
                            stopWhy = f"offset {cs['d']*100:+.1f} cm"
                    if stopWhy:
                        print(f"STOPPED (camera steering): {stopWhy}. s = {cs['s']:.2f} m, "
                              f"offset {cs['d']*100:+.1f} cm, heading {np.degrees(cs['h']):+.1f} deg  "
                              f"t_epoch={time.time():.2f}")
                        break

            v = qcar.motorTach

            # Odometry log (no effect on driving)
            if odoFile is not None:
                gzRaw = float(qcar.gyroscope[2])
                tach = float(qcar.motorTach[0]) if np.ndim(qcar.motorTach) else float(qcar.motorTach)
                enc = int(qcar.motorEncoder[0])
                if odoEnc0 is None:
                    odoEnc0 = enc
                if t < startDelay:
                    odoBiasSum += gzRaw
                    odoBiasN += 1
                elif odoBias is None:
                    odoBias = odoBiasSum / max(odoBiasN, 1)
                    print(f"ODOMETRY: gyro bias {odoBias:+.5f} rad/s from {odoBiasN} samples")
                gz = gzRaw - (odoBias or 0.0)
                if odoBias is not None:
                    odoHeading += gz * dt
                odoDistTach += tach * dt
                if t - odoLast >= odo_log_period:
                    odoLast = t
                    e = enc - odoEnc0
                    csTxt = (f"{cs['s']:.4f},{cs['d']*100:.2f},{np.degrees(cs['h']):.2f},{cs['src']}"
                             if cs is not None else "nan,nan,nan,")
                    odoFile.write(f"{time.time():.3f},{e},{e * cpsToMps:.4f},{tach:.4f},{odoDistTach:.4f},"
                                  f"{gz:.5f},{np.degrees(odoHeading):.2f},{gzRaw:.5f},"
                                  f"{x:.3f},{y:.3f},{np.degrees(th):.2f},{delta:.4f},{csTxt}\n")
            #endregion

            #region : Update controllers and write to car
            if t < startDelay:
                u = 0
                delta = 0
            else:
                #region : Speed controller update
                u = speedController.update(v, vTarget*vGain, dt)
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
                        activeController = initSteeringController
                    else:
                        delta = steeringController.update(p_steer, thSteer, v, tanRef)
                        activeController = steeringController
                    # pose= line about twice per second, also while the start
                    # controller is active (2026-11-17: a whole 0->2->4 drive ran on
                    # it and logged no pose= lines, camera report §21)
                    if int(t*2) != int((t-dt)*2):
                        camTxt = ""
                        if cam is not None:
                            camTxt = (f"  cam={cam.mode}/{cam.offset*100:+.1f}cm  "
                                      f"bias={camBias*100:+.1f}cm  used={int(camUsed)}")
                        if cs is not None:
                            camTxt = (f"  cs: s={cs['s']:.2f} m  offset={cs['d']*100:+.1f}cm  "
                                      f"heading={np.degrees(cs['h']):+.1f}deg  src={cs['src']}  "
                                      f"fits={cs['nFit']}  rej={cs['nRej']}  wait={cs['nWait']}  hbad={cs['nHbad']}  gps_s={cs['nGps']}  shift={cs['nShift']}  "
                                      + (f"gps_rej={cs['nGpsRej']}  sd_d={100*fus.sd()[1]:.1f}cm  sd_s={100*fus.sd()[0]:.1f}cm  " if fus is not None else "")
                                      + f"v_target={vTarget:.2f}")
                        initTxt = "" if startNodeReached else "  init=1"
                        print(f"pose=({x:.2f},{y:.2f},{np.degrees(th):.0f}deg)  path_pt=({activeController.p_ref[0]:.2f},{activeController.p_ref[1]:.2f})  cross_track={activeController.ect:.3f} m{camTxt}{initTxt}  t_epoch={time.time():.2f}")
                else:
                    delta = 0
                #endregion
            qcar.write(u, delta + steering_trim)            
            # end = elapsed_time()
            continue
        qcar.read_write_std(throttle= 0, steering= 0)
        if odoFile is not None:
            odoFile.close()
            print(f"ODOMETRY: saved {odo_log_file}")

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
