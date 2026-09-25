# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --

#region : File Description and Imports

"""
vehicle_control.py

Skills acivity code for vehicle control lab guide.
Students will implement a vehicle speed and steering controller.
Please review Lab Guide - vehicle control PDF
"""
import signal
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
from road_rules import RoadNetwork
from intersection_client import IntersectionClient, IntersectionController
from localization_diagnostics import LocalizationDiagnostics
from path_manager import PathManager
from tracking_diagnostics import PathCurvatureAnalyzer, TrackingDiagnostics
from curvature_control import CurvatureAwareControl
from footprint_correction import FootprintReferenceCorrection

parser = argparse.ArgumentParser(prog='Vehicle control')
parser.add_argument('-c','--calibrate', default=False)
parser.add_argument('-n', '--node_configuration', type=int, choices=(0, 1), default=0)
parser.add_argument('-s', '--start_node', type=int, default=None)
parser.add_argument('--coordinator_ip', default=None)
parser.add_argument('--coordinator_port', type=int, default=18750)
parser.add_argument('--car_id', default=None)
parser.add_argument(
    '--curvature_control',
    action='store_true',
    help='Enable bounded curvature feedforward and curve-speed scheduling.',
)
parser.add_argument(
    '--tracking_offset_m',
    type=float,
    default=0.20,
    help=(
        'Distance forward from the EKF pose used as the Stanley tracking '
        'point (default: 0.20 m).'
    ),
)
parser.add_argument(
    '--footprint_correction',
    action='store_true',
    help=(
        'Enable bounded curvature-based outward reference correction for '
        'vehicle off-tracking.'
    ),
)
parser.add_argument('--footprint_correction_gain', type=float, default=1.0)
parser.add_argument('--footprint_correction_max_m', type=float, default=0.04)

args = parser.parse_args()
if not 0.0 <= args.tracking_offset_m <= 0.30:
    parser.error('--tracking_offset_m must be between 0.0 and 0.30 m')
if not 0.0 < args.footprint_correction_gain <= 3.0:
    parser.error('--footprint_correction_gain must be in (0.0, 3.0]')
if not 0.0 <= args.footprint_correction_max_m <= 0.08:
    parser.error('--footprint_correction_max_m must be between 0.0 and 0.08 m')
node_options = args.node_configuration
roadNetwork = RoadNetwork()

# choosing between two sets of nodes which avoids two way traffic
if node_options==0:
    valid_nodes=[10,2,4,14,20,22,9,7,0]
    # valid_nodes=[0,2,4,6,13,19,17,15,5,3,1]
else:
    valid_nodes=[10,1,5,3,8,23,21,15,6]
    # valid_nodes=[11,12,7,14,16,18]

# Previous randomized route selection. Keep this commented out until every
# possible transition has been verified on the physical map.
# random.shuffle(valid_nodes)
# num_nodes = random.randint(6, min(10, len(valid_nodes)))
# route_nodes = valid_nodes[:num_nodes]
# if args.start_node is None:
#     nodeSequence = route_nodes + [route_nodes[0]]
# else:
#     route_nodes = [node for node in route_nodes if node != args.start_node]
#     nodeSequence = [args.start_node] + route_nodes + [args.start_node]

# Fixed, reproducible routes for the current physical setup.
if args.calibrate:
    # Calibration does not drive this route, but waypoint initialization still
    # requires a valid sequence.
    nodeSequence = [11, 6, 11]
elif args.start_node == 11:
    nodeSequence = list(roadNetwork.get_route("START_NODE_11"))
elif args.start_node == 0:
    nodeSequence = list(roadNetwork.get_route("START_NODE_0"))
else:
    raise ValueError(
        f"No fixed route configured for start node {args.start_node}."
    )

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
v_ref = 0.25
K_p = 0.1
K_i = 1

# ===== Steering Controller Parameters
# - enableSteeringControl: whether or not to enable steering control
# - K_stanley: K gain for stanley controller
# - nodeSequence: list of nodes from roadmap. Used for trajectory generation.
enableSteeringControl = True
K_stanley = 0.7

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
if enableSteeringControl:
    roadmap = SDCSRoadMap(leftHandTraffic=False)
    rawWaypointSequence = roadmap.generate_path(nodeSequence)
    pathResult = PathManager(
        waypoints=rawWaypointSequence,
        node_sequence=nodeSequence,
        roadmap=roadmap,
    ).prepare()
    for repair in pathResult.repairs:
        print(
            "[PATH] Repaired paired heading reversal: "
            f"removed waypoint {repair['removed_index']} at "
            f"{repair['removed_point']}, turns="
            f"({repair['first_turn_degrees']:.2f}, "
            f"{repair['second_turn_degrees']:.2f}) deg",
            flush=True,
        )
    for warning in pathResult.warnings:
        print(f"[PATH] Warning: {warning}", flush=True)
    if not pathResult.valid:
        for error in pathResult.errors:
            print(f"[PATH] ERROR: {error}", flush=True)
        raise RuntimeError(
            "Generated route failed path validation; vehicle will not start."
        )
    waypointSequence = pathResult.waypoints
    print(
        f"[PATH] Validated {waypointSequence.shape[1]} waypoints; "
        f"repairs={len(pathResult.repairs)}",
        flush=True,
    )
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
        # A stop command must not retain positive integral throttle from
        # cruising, otherwise the QCar can creep across its stop line.
        if v_ref <= 0.01:
            self.ei = 0
            return 0.0

        e = v_ref - v
        self.ei += dt*e

        return np.clip(
            self.kp*e + self.ki*self.ei,
            -self.maxThrottle,
            self.maxThrottle
        )
    
        return 0

class SteeringController:

    def __init__(
        self,
        waypoints,
        k=1,
        cyclic=True,
        curvature_control=False,
        footprint_correction=False,
        footprint_correction_gain=1.0,
        footprint_correction_max_m=0.04,
    ):
        self.maxSteeringAngle = np.pi/6

        self.wp = waypoints
        self.N = len(waypoints[0, :])
        self.wpi = 0

        self.k = k
        self.cyclic = cyclic

        self.p_ref = (0, 0)
        self.th_ref = 0
        self.cross_track_error = 0.0
        self.heading_error = 0.0
        self.distance_to_next_waypoint = float("inf")
        self.control_mode = "curvature_feedback_v3" if curvature_control else "baseline"
        self.feedforward_steering = 0.0
        self.feedback_steering = 0.0
        self.active_heading_gain = 1.0
        self.curve_speed_target = v_ref
        self.curvatureController = None
        self.curvatureAnalyzer = None
        self.footprintCorrection = None
        self.footprint_correction_target = 0.0
        self.footprint_correction_applied = 0.0
        if curvature_control or footprint_correction:
            self.curvatureAnalyzer = PathCurvatureAnalyzer(waypoints)
        if curvature_control:
            self.curvatureController = CurvatureAwareControl(
                analyzer=self.curvatureAnalyzer,
                maximum_steering_angle=self.maxSteeringAngle,
                cross_track_gain=self.k,
            )
        if footprint_correction:
            self.footprintCorrection = FootprintReferenceCorrection(
                correction_gain=footprint_correction_gain,
                maximum_offset=footprint_correction_max_m,
            )
            self.control_mode += "+footprint_reference_v2"

    # ==============  SECTION B -  Steering Control  ====================
    def update(self, p, th, speed, dt=0.0):
        wp_1 = self.wp[:, np.mod(self.wpi, self.N-1)]
        wp_2 = self.wp[:, np.mod(self.wpi+1, self.N-1)]
        v = wp_2 - wp_1
        v_mag = np.linalg.norm(v)
        if v_mag <= 1e-9:
            return 0.0
        v_uv = v / v_mag
        s = np.dot(p-wp_1, v_uv)
        distance_to_next = float(np.linalg.norm(p - wp_2))
        self.distance_to_next_waypoint = distance_to_next

        # Baseline SDCS waypoint progression. Keep the additional measurements
        # below for diagnosis, but do not change tracking behavior until the
        # baseline has been characterized on both physical vehicles.
        if s >= v_mag:
            if self.cyclic or self.wpi < self.N - 2:
                self.wpi += 1

        tangent = np.arctan2(v_uv[1], v_uv[0])

        ep = wp_1 + v_uv*s
        if self.footprintCorrection is not None:
            curvature = self.curvatureAnalyzer.metrics(self.wpi)[
                "local_curvature"
            ]
            lateral_offset = self.footprintCorrection.update(curvature, dt)
            left_normal = np.array([-v_uv[1], v_uv[0]])
            ep = ep + lateral_offset * left_normal
            self.footprint_correction_target = (
                self.footprintCorrection.target_offset
            )
            self.footprint_correction_applied = lateral_offset
        ct = ep - p
        dir = wrap_to_pi(np.arctan2(ct[1], ct[0]) - tangent)

        ect = np.linalg.norm(ct) * np.sign(dir)
        psi = wrap_to_pi(tangent-th)

        self.p_ref = ep
        self.th_ref = tangent
        self.cross_track_error = float(ect)
        self.heading_error = float(psi)

        if self.curvatureController is not None:
            steering = self.curvatureController.update(
                waypoint_index=self.wpi,
                heading_error=psi,
                cross_track_error=ect,
                speed=speed,
                dt=dt,
            )
            self.feedforward_steering = (
                self.curvatureController.filtered_feedforward
            )
            self.feedback_steering = (
                self.curvatureController.feedback_steering
            )
            self.active_heading_gain = (
                self.curvatureController.active_heading_gain
            )
            return steering

        self.feedback_steering = wrap_to_pi(
            psi + np.arctan2(self.k*ect, speed)
        )
        return np.clip(
            self.feedback_steering,
            -self.maxSteeringAngle,
            self.maxSteeringAngle)

    def recommended_speed(self, nominal_speed):
        if self.curvatureController is None:
            self.curve_speed_target = float(nominal_speed)
        else:
            self.curve_speed_target = self.curvatureController.recommended_speed(
                nominal_speed,
                self.wpi,
            )
        return self.curve_speed_target
        
        # return 0

def controlLoop():
    #region controlLoop setup
    global KILL_THREAD
    u = 0
    delta = 0
    commandedSpeed = 0.0
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
            curvature_control=args.curvature_control,
            footprint_correction=args.footprint_correction,
            footprint_correction_gain=args.footprint_correction_gain,
            footprint_correction_max_m=args.footprint_correction_max_m,
        )
        print(
            f"[CONTROL] Steering mode: {steeringController.control_mode}; "
            f"tracking offset={args.tracking_offset_m:.3f} m; "
            f"footprint correction={args.footprint_correction}; "
            f"footprint gain={args.footprint_correction_gain:.2f}; "
            f"footprint max={args.footprint_correction_max_m:.3f} m",
            flush=True,
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

    # Check the initial pose of the car. LiDAR localization can publish an
    # outlier while it is converging, so wait briefly for a fix near the
    # declared physical start node instead of accepting the first packet.
    expectedStartPose = None
    if args.start_node is not None:
        expectedStartPose = roadmap.get_node_pose(args.start_node).squeeze()
    print("[INFO] Waiting for initial GPS/LiDAR fix...")
    new = False
    initPose = None
    fixDeadline = time.time() + 8.0
    rejectedFixes = 0
    while not KILL_THREAD and time.time() < fixDeadline:
        new = gps.readGPS()
        if new:
            candidatePose = np.array([
                gps.position[0],
                gps.position[1],
                gps.orientation[2],
            ])
            if calibrate or expectedStartPose is None:
                initPose = candidatePose
                break
            candidateError = np.linalg.norm(
                candidatePose[:2] - expectedStartPose[:2]
            )
            initPose = candidatePose
            if candidateError <= 0.75:
                break
            rejectedFixes += 1
            print(
                f"[WARN] Rejected localization fix "
                f"x={candidatePose[0]:.2f}, y={candidatePose[1]:.2f} "
                f"({candidateError:.2f} m from start node); waiting for "
                "localization to converge...",
                flush=True,
            )
        time.sleep(0.01)
    if KILL_THREAD:
        qcar.terminate()
        gps.terminate()
        yolo.terminate()
        return
    if initPose is None:
        print("[ERROR] No GPS/LiDAR fix received within 8 seconds.", flush=True)
        qcar.terminate()
        gps.terminate()
        yolo.terminate()
        return
    print(
        f"[INFO] GPS fix acquired: x={initPose[0]:.2f}, "
        f"y={initPose[1]:.2f}, heading={initPose[2]:.2f} "
        f"after rejecting {rejectedFixes} outlier(s)",
        flush=True,
    )

    # Calibration only needs GPS/LiDAR initialization. In particular, do not
    # call initial_check(), which is unavailable in the older HAL on .151.
    if calibrate:
        qcar.terminate()
        gps.terminate()
        yolo.terminate()
        return

    if args.start_node is not None:
        startPose = expectedStartPose
        distanceToStart = np.linalg.norm(initPose[:2] - startPose[:2])
        headingError = abs(wrap_to_pi(initPose[2] - startPose[2]))
        print(
            f"[INFO] Start node {args.start_node}: "
            f"position error={distanceToStart:.2f} m, "
            f"heading error={np.degrees(headingError):.1f} deg",
            flush=True,
        )
        if distanceToStart > 0.5 or headingError > np.pi/3:
            print(
                "[ERROR] Start pose does not match the declared start node. "
                "The controller will remain stopped. Reposition and orient "
                "the QCar, then run again.",
                flush=True,
            )
            qcar.terminate()
            gps.terminate()
            yolo.terminate()
            return

        # Since this route begins at the declared physical node, a recovery
        # path is unnecessary. This works with both old and new HAL versions.
        startNodeReached = True
    elif hasattr(roadmap, 'initial_check'):
        startNodeReached, initWaypointSequence = roadmap.initial_check(
            initPose,
            nodeSequence,
            waypointSequence,
        )
        if not startNodeReached:
            initSteeringController = SteeringController(
                waypoints=initWaypointSequence,
                k=K_stanley,
                cyclic=False,
            )
    else:
        print(
            "[ERROR] This QCar's HAL library does not support automatic "
            "route recovery. Launch with --start_node.",
            flush=True,
        )
        qcar.terminate()
        gps.terminate()
        yolo.terminate()
        return

    if not args.coordinator_ip or not args.car_id:
        print(
            "[ERROR] Normal driving requires --coordinator_ip and --car_id.",
            flush=True,
        )
        qcar.terminate()
        gps.terminate()
        yolo.terminate()
        return

    route_id = f"START_NODE_{args.start_node}"
    zone_id = "CZ1_CENTRAL"
    routeConfig = roadNetwork.get_route_config(route_id)
    approach_id = routeConfig["intersection_approaches"][zone_id]
    reservationClient = IntersectionClient(
        coordinator_ip=args.coordinator_ip,
        port=args.coordinator_port,
        car_id=args.car_id,
        zone_id=zone_id,
    )
    intersectionController = IntersectionController(
        road_network=roadNetwork,
        zone_id=zone_id,
        approach_id=approach_id,
        client=reservationClient,
        base_speed=v_ref,
    )
    print(
        f"[INTERSECTION] Configured {zone_id}/{approach_id}; "
        f"coordinator={args.coordinator_ip}:{args.coordinator_port}",
        flush=True,
    )
    localizationDiagnostics = LocalizationDiagnostics(car_id=args.car_id)
    trackingDiagnostics = TrackingDiagnostics(
        car_id=args.car_id,
        route_id=route_id,
        waypoints=waypointSequence,
        maximum_steering_angle=steeringController.maxSteeringAngle,
    )

    with qcar, gps, yolo:
        t0 = time.time()
        t=0
        startTime = time.time()
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
            yoloGain = yoloDrive.check_yolo(yolo.stopSign,
                                            yolo.trafficlight,
                                            yolo.cars,
                                            yolo.yieldSign,
                                            yolo.person)
            if enableSteeringControl:

                if gps.readLidar():
                    localizationDiagnostics.update_lidar(
                        t,
                        gps.angles,
                        gps.distances,
                    )

                gpsNew = gps.readGPS()
                if gpsNew:
                    y_gps = np.array([
                        gps.position[0],
                        gps.position[1],
                        gps.orientation[2]
                    ])
                    ekfPre = ekf.x_hat[:, 0].copy()
                    gpsPositionInnovation = np.linalg.norm(
                        y_gps[:2] - ekfPre[:2]
                    )
                    gpsHeadingInnovation = wrap_to_pi(
                        y_gps[2] - ekfPre[2]
                    )
                    ekf.update(
                        [qcar.motorTach, delta],
                        dt,
                        y_gps,
                        qcar.gyroscope[2],
                    )
                    localizationDiagnostics.record_gps_update(
                        elapsed=t,
                        raw_pose=y_gps,
                        ekf_pre=ekfPre,
                        ekf_post=ekf.x_hat[:, 0],
                        position_innovation=gpsPositionInnovation,
                        heading_innovation=gpsHeadingInnovation,
                        motor_speed=qcar.motorTach,
                        steering_command=delta,
                        gyro_z=qcar.gyroscope[2],
                    )
                else:
                    ekf.update(
                        [qcar.motorTach, delta],
                        dt,
                        None,
                        qcar.gyroscope[2],
                    )

                x = ekf.x_hat[0,0]
                y = ekf.x_hat[1,0]
                th = ekf.x_hat[2,0]
                p = ( np.array([x, y])
                    + np.array([np.cos(th), np.sin(th)])
                    * args.tracking_offset_m)
                
            v = qcar.motorTach
            intersectionGain = intersectionController.update(p, v)
            vGain = min(yoloGain, intersectionGain)
            #endregion

            #region : Update controllers and write to car
            if t < startDelay:
                u = 0
                delta = 0
                commandedSpeed = 0.0
            else:
                #region : Steering controller update
                if enableSteeringControl:
                    if not startNodeReached:
                        # check the criterion inside the if condition to ensure it will
                        # only be executed once
                        distToStart = np.linalg.norm(waypointSequence[:,0]-p)
                        startNodeReached = distToStart < 0.2
                        # if the start node is not reached, use the initial steering controller
                        delta = initSteeringController.update(p, th, v, dt)
                    else:
                        delta = steeringController.update(p, th, v, dt)
                else:
                    delta = 0
                #endregion

                #region : Speed controller update
                curveSpeed = steeringController.recommended_speed(v_ref)
                commandedSpeed = curveSpeed * vGain
                u = speedController.update(v, commandedSpeed, dt)
                #endregion
            if t < startDelay:
                vGain = 0.0
            qcar.write(u, delta)            
            if enableSteeringControl and startNodeReached and t >= startDelay:
                trackingDiagnostics.record(
                    elapsed=t,
                    pose=[x, y, th],
                    tracking_point=p,
                    controller=steeringController,
                    speed=v,
                    target_speed=commandedSpeed,
                    speed_gain=vGain,
                    throttle=u,
                    steering=delta,
                    intersection_phase=intersectionController.phase,
                )
            # end = elapsed_time()
            if int(t * 2) != int(tp * 2):
                print(
                    f"[STATUS] speed={v:.2f} m/s, vGain={vGain:.2f}, "
                    f"throttle={u:.3f}, steering={delta:.3f}, "
                    f"intersection={intersectionController.phase}, "
                    f"stopDist={intersectionController.distance_to_stop:.2f}, "
                    f"inZone={intersectionController.in_conflict}, "
                    f"inExit={intersectionController.in_exit}, "
                    f"position=({x:.3f},{y:.3f}), "
                    f"rawGPS=({localizationDiagnostics.last_raw_pose[0]:.3f},"
                    f"{localizationDiagnostics.last_raw_pose[1]:.3f}), "
                    f"gpsInnovation="
                    f"{localizationDiagnostics.last_position_innovation:.3f} m, "
                    f"gpsAge={localizationDiagnostics.gps_age(t):.2f} s, "
                    f"lidarValid={localizationDiagnostics.last_lidar_valid}, "
                    f"wp={steeringController.wpi}, "
                    f"wpDistance="
                    f"{steeringController.distance_to_next_waypoint:.3f} m, "
                    f"control={steeringController.control_mode}, "
                    f"curveSpeed={steeringController.curve_speed_target:.2f}, "
                    f"feedforward="
                    f"{steeringController.feedforward_steering:.3f}, "
                    f"feedback={steeringController.feedback_steering:.3f}, "
                    f"headingGain={steeringController.active_heading_gain:.2f}, "
                    f"footprintTarget="
                    f"{steeringController.footprint_correction_target:.3f} m, "
                    f"footprintApplied="
                    f"{steeringController.footprint_correction_applied:.3f} m, "
                    f"crossTrack={steeringController.cross_track_error:.3f} m, "
                    f"headingError="
                    f"{np.degrees(steeringController.heading_error):.1f} deg",
                    flush=True,
                )
            continue
        qcar.read_write_std(throttle= 0, steering= 0)
    localizationDiagnostics.close()
    trackingDiagnostics.close()
    intersectionController.close()

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
