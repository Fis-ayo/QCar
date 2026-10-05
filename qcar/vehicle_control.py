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
from pal.products.qcar import QCar, QCarGPS

from pal.utilities.math import wrap_to_pi
from hal.content.qcar_functions import QCarEKF
from hal.products.mats import SDCSRoadMap
import argparse
from utils import YOLOReceiver, YOLODriveLogic
from road_rules import RoadNetwork
from intersection_client import IntersectionClient, IntersectionController
from localization_diagnostics import LocalizationDiagnostics
from path_manager import PathManager
from path_tracking import SmoothPolylineTracker
from control_modes import MODES, get_control_mode
from control_safety import (
    LocalizationFreshnessMonitor,
    SafeSpeedController,
    TrackingLossMonitor,
)
from tracking_diagnostics import TrackingDiagnostics
from lane_guidance import (
    LANE_GUIDANCE_PORT,
    BoundedHybridSteering,
    ContinuousCorridorAdvisor,
    FusedLaneShadowAdvisor,
    HybridLaneAdvisor,
    LaneCalibrationModel,
    LaneGuidanceReceiver,
    unavailable_observation,
)
from lane_safety import (
    LANE_SAFETY_PORT,
    BoundaryCategorySafetyMonitor,
    LaneSafetyReceiver,
)

parser = argparse.ArgumentParser(prog='Vehicle control')
parser.add_argument('-c','--calibrate', default=False)
parser.add_argument('-s', '--start_node', type=int, default=None)
parser.add_argument('--coordinator_ip', default=None)
parser.add_argument('--coordinator_port', type=int, default=18750)
parser.add_argument('--car_id', default=None)
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
    '--lane_safety',
    action='store_true',
    help='Enable CSI-based speed slowdown/latched stop; never changes steering.',
)
parser.add_argument('--lane_safety_port', type=int, default=LANE_SAFETY_PORT)
parser.add_argument(
    '--lane_guidance_shadow',
    action='store_true',
    help='Log bounded CSI steering proposals with zero actuator authority.',
)
parser.add_argument(
    '--control_mode',
    choices=tuple(MODES),
    default='baseline_safety',
    help='Explicit actuator policy: baseline, baseline_safety, or hybrid.',
)
parser.add_argument(
    '--allow_unsafe_baseline',
    action='store_true',
    help='Acknowledge that historical baseline can reverse and has no guards.',
)
parser.add_argument('--lane_guidance_port', type=int, default=LANE_GUIDANCE_PORT)
parser.add_argument('--start_position_tolerance_m', type=float, default=0.25)
parser.add_argument('--start_heading_tolerance_deg', type=float, default=30.0)

args = parser.parse_args()
if not 0.0 <= args.tracking_offset_m <= 0.30:
    parser.error('--tracking_offset_m must be between 0.0 and 0.30 m')
controlMode = get_control_mode(args.control_mode)
if controlMode.name == 'baseline' and not args.allow_unsafe_baseline:
    parser.error(
        'historical baseline is replay/reference only; pass '
        '--allow_unsafe_baseline for an explicitly supervised reproduction'
    )
if controlMode.camera_steering and not args.lane_guidance_shadow:
    parser.error('hybrid control mode requires --lane_guidance_shadow')
if args.start_position_tolerance_m <= 0.0:
    parser.error('--start_position_tolerance_m must be positive')
if not 0.0 < args.start_heading_tolerance_deg <= 90.0:
    parser.error('--start_heading_tolerance_deg must be in (0, 90]')
roadNetwork = RoadNetwork()

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
        if controlMode.name == "baseline":
            print(
                "[CONTROL WARNING] Historical baseline is unsafe: signed PI "
                "output can command reverse torque and no new emergency "
                "guards have authority.",
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
        self.state = "NORMAL"


    # ==============  SECTION A -  Speed Control  ====================
    def update(self, v, v_ref, dt):
        # A stop command must not retain positive integral throttle from
        # cruising, otherwise the QCar can creep across its stop line.
        if v_ref <= 0.01:
            self.ei = 0
            self.state = "STOPPED"
            return 0.0

        self.state = "NORMAL"
        e = v_ref - v
        self.ei += dt*e

        return np.clip(
            self.kp*e + self.ki*self.ei,
            -self.maxThrottle,
            self.maxThrottle
        )
    
        return 0

    @property
    def integral_error(self):
        return self.ei

class SteeringController:

    def __init__(
        self,
        waypoints,
        k=1,
        cyclic=True,
        minimum_control_speed=None,
    ):
        self.maxSteeringAngle = np.pi/6

        self.wp = waypoints
        self.N = len(waypoints[0, :])
        self.wpi = 0

        self.k = k
        self.cyclic = cyclic
        self.minimum_control_speed = minimum_control_speed

        self.p_ref = (0, 0)
        self.th_ref = 0
        self.cross_track_error = 0.0
        self.heading_error = 0.0
        self.distance_to_next_waypoint = float("inf")
        self.feedback_steering = 0.0

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
        ct = ep - p
        dir = wrap_to_pi(np.arctan2(ct[1], ct[0]) - tangent)

        ect = np.linalg.norm(ct) * np.sign(dir)
        psi = wrap_to_pi(tangent-th)

        self.p_ref = ep
        self.th_ref = tangent
        self.cross_track_error = float(ect)
        self.heading_error = float(psi)

        controlSpeed = float(speed)
        if self.minimum_control_speed is not None:
            controlSpeed = max(
                abs(controlSpeed), float(self.minimum_control_speed)
            )
        self.feedback_steering = wrap_to_pi(
            psi + np.arctan2(self.k*ect, controlSpeed)
        )
        return np.clip(
            self.feedback_steering,
            -self.maxSteeringAngle,
            self.maxSteeringAngle)

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
    speedController = (
        SafeSpeedController(kp=K_p, ki=K_i)
        if controlMode.safe_speed_control
        else SpeedController(kp=K_p, ki=K_i)
    )
    if enableSteeringControl:
        steeringController = SteeringController(
            waypoints=waypointSequence,
            k=K_stanley,
            minimum_control_speed=(
                0.15 if controlMode.regularized_stanley_speed else None
            ),
        )
        smoothPathTracker = SmoothPolylineTracker(
            waypoints=waypointSequence,
            stanley_gain=K_stanley,
            maximum_steering_rad=steeringController.maxSteeringAngle,
        )
        smoothPathShadow = smoothPathTracker.unavailable(0)
        print(
            f"[CONTROL MODE] {controlMode.name}; "
            f"tracking offset={args.tracking_offset_m:.3f} m",
            flush=True,
        )
        print(
            "[CONTROL AUTHORITY] "
            f"speed={'safe' if controlMode.safe_speed_control else 'original'}, "
            f"steering={'waypoint+camera' if controlMode.camera_steering else 'waypoint-only'}, "
            f"localizationStop={controlMode.localization_stop}, "
            f"trackingStop={controlMode.tracking_loss_stop}, "
            "boundarySpeed=False",
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

    # Calibration only needs GPS/LiDAR initialization and never enters the
    # route-validation or driving stages.
    if calibrate:
        qcar.terminate()
        gps.terminate()
        yolo.terminate()
        return

    startPose = expectedStartPose
    distanceToStart = np.linalg.norm(initPose[:2] - startPose[:2])
    headingError = abs(wrap_to_pi(initPose[2] - startPose[2]))
    print(
        f"[INFO] Start node {args.start_node}: "
        f"position error={distanceToStart:.2f} m, "
        f"heading error={np.degrees(headingError):.1f} deg",
        flush=True,
    )
    if (
        distanceToStart > args.start_position_tolerance_m
        or headingError > np.deg2rad(args.start_heading_tolerance_deg)
    ):
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
    localizationFreshnessMonitor = LocalizationFreshnessMonitor()
    localizationSafety = localizationFreshnessMonitor.update(float("inf"))
    lastLocalizationSafetyState = localizationSafety["state"]
    trackingLossMonitor = TrackingLossMonitor()
    trackingLossSafety = trackingLossMonitor.update(0.0, 0.0, 0.0)
    lastTrackingLossState = trackingLossSafety["state"]
    trackingDiagnostics = TrackingDiagnostics(
        car_id=args.car_id,
        route_id=route_id,
        waypoints=waypointSequence,
        maximum_steering_angle=steeringController.maxSteeringAngle,
    )
    laneSafetyReceiver = None
    laneSafety = {
        "state": "DISABLED",
        "speed_gain": 1.0,
        "reliable": False,
        "confidence": float("nan"),
        "offset": float("nan"),
        "age_s": float("nan"),
    }
    lastLaneSafetyState = laneSafety["state"]
    if args.lane_safety:
        laneSafetyReceiver = LaneSafetyReceiver(
            port=args.lane_safety_port,
            expected_car_id=args.car_id,
        )
        print(
            f"[LANE SAFETY] Enabled speed-only intervention on local UDP "
            f"port {args.lane_safety_port}; critical stops latch until restart.",
            flush=True,
        )
    laneGuidanceReceiver = None
    laneGuidanceObservation = unavailable_observation()
    laneCalibrationModel = None
    if args.lane_guidance_shadow:
        laneCalibrationModel = LaneCalibrationModel.from_file(
            "lane_calibration.json"
        )
    laneGuidanceAdvisor = HybridLaneAdvisor(
        allow_actuation=controlMode.camera_steering,
        allow_calibrated_actuation=controlMode.camera_steering,
        calibration_model=laneCalibrationModel,
        maximum_correction_rad=0.02,
        maximum_slew_rad_s=0.05,
    )
    laneGuidance = laneGuidanceAdvisor.update(
        laneGuidanceObservation,
        0.0,
    )
    fusedLaneShadowAdvisor = FusedLaneShadowAdvisor(
        boundary_side=(
            laneCalibrationModel.boundary_side
            if laneCalibrationModel is not None else "left"
        ),
        maximum_correction_rad=0.01,
        maximum_steering_rad=steeringController.maxSteeringAngle,
        allow_actuation=False,
    )
    fusedLaneShadow = fusedLaneShadowAdvisor.update(laneGuidance, 0.0)
    continuousCorridorAdvisor = ContinuousCorridorAdvisor()
    continuousCorridor = continuousCorridorAdvisor.update(
        laneGuidance, 0.0, 0.0, 0.0
    )
    boundedHybridSteering = BoundedHybridSteering(
        boundary_side=(
            laneCalibrationModel.boundary_side
            if laneCalibrationModel is not None else "left"
        ),
        maximum_correction_rad=0.02,
        maximum_slew_rad_s=0.05,
        maximum_steering_rad=steeringController.maxSteeringAngle,
    )
    boundarySteering = {
        "state": "DISABLED",
        "unconstrained_combined_rad": 0.0,
        "constrained_waypoint_rad": 0.0,
        "protective_correction_rad": 0.0,
        "constraint_adjustment_rad": 0.0,
        "final_steering_rad": 0.0,
    }
    lastBoundarySteeringState = boundarySteering["state"]
    boundaryCategoryMonitor = BoundaryCategorySafetyMonitor()
    boundaryCategorySafety = boundaryCategoryMonitor.snapshot()
    lastBoundaryCategoryState = boundaryCategorySafety["state"]
    if args.lane_guidance_shadow:
        laneGuidanceReceiver = LaneGuidanceReceiver(
            port=args.lane_guidance_port,
            expected_car_id=args.car_id,
        )
        if controlMode.camera_steering:
            print(
                "[HYBRID] EXPERIMENTAL low-authority camera steering enabled; "
                "calibrated correction capped at 0.020 rad with 0.050 rad/s "
                "slew limit.",
                flush=True,
            )
        else:
            print(
                "[HYBRID] Camera guidance SHADOW mode enabled; "
                "applied steering authority is fixed at 0.00.",
                flush=True,
            )
        print(
            "[BOUNDARY SAFETY] SHADOW mode; states are logged but cannot "
            "reduce commanded speed.",
            flush=True,
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
            if laneSafetyReceiver is not None:
                laneSafety = laneSafetyReceiver.receive()
                if laneSafety["state"] != lastLaneSafetyState:
                    print(
                        f"[LANE SAFETY] {lastLaneSafetyState} -> "
                        f"{laneSafety['state']}; "
                        f"gain={laneSafety['speed_gain']:.2f}, "
                        f"confidence={laneSafety['confidence']:.2f}, "
                        f"offset={laneSafety['offset']:.3f}, "
                        f"age={laneSafety['age_s']:.3f} s",
                        flush=True,
                    )
                    lastLaneSafetyState = laneSafety["state"]
            if laneGuidanceReceiver is not None:
                laneGuidanceObservation = laneGuidanceReceiver.receive()
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
            localizationSafety = localizationFreshnessMonitor.update(
                localizationDiagnostics.gps_age(t)
            )
            if localizationSafety["state"] != lastLocalizationSafetyState:
                print(
                    f"[LOCALIZATION SAFETY] {lastLocalizationSafetyState} -> "
                    f"{localizationSafety['state']}; "
                    f"gpsAge={localizationSafety['gps_age_s']:.3f} s",
                    flush=True,
                )
                lastLocalizationSafetyState = localizationSafety["state"]
            intersectionGain = intersectionController.update(p, v)
            # Normal driving authority has two established sources. Camera
            # safety remains diagnostic-only in the cleaned experiment modes.
            vGain = min(yoloGain, intersectionGain)
            if controlMode.localization_stop:
                vGain = min(vGain, localizationSafety["speed_gain"])
            #endregion

            #region : Update controllers and write to car
            if t < startDelay:
                u = 0
                delta = 0
                commandedSpeed = 0.0
            else:
                #region : Steering controller update
                if enableSteeringControl:
                    delta = steeringController.update(p, th, v, dt)
                else:
                    delta = 0
                #endregion

                waypointSteering = float(delta)
                smoothPathShadow = smoothPathTracker.update(
                    p,
                    th,
                    v,
                    steeringController.wpi,
                )
                if laneGuidanceReceiver is not None:
                    laneGuidanceObservation = dict(laneGuidanceObservation)
                    laneGuidanceObservation["path_curvature_inv_m"] = (
                        trackingDiagnostics.curvature_analyzer.metrics(
                            steeringController.wpi
                        )["local_curvature"]
                    )
                laneGuidance = laneGuidanceAdvisor.update(
                    laneGuidanceObservation,
                    dt,
                )
                pathCurvature = trackingDiagnostics.curvature_analyzer.metrics(
                    steeringController.wpi
                )["local_curvature"]
                boundaryCategorySafety = boundaryCategoryMonitor.update(
                    laneGuidance["boundary_category"],
                    laneGuidance["yellow_reference_x_px"],
                    dt,
                    laneGuidance["physical_lateral_error_m"],
                )
                if boundaryCategorySafety["state"] != lastBoundaryCategoryState:
                    print(
                        f"[BOUNDARY SAFETY] {lastBoundaryCategoryState} -> "
                        f"{boundaryCategorySafety['state']}; "
                        f"category={laneGuidance['boundary_category']}, "
                        f"gain={boundaryCategorySafety['speed_gain']:.2f}, "
                        f"riskTime={boundaryCategorySafety['risk_duration_s']:.2f} s, "
                        f"inwardProgress="
                        f"{boundaryCategorySafety['inward_progress_px']:.1f} px",
                        flush=True,
                    )
                    lastBoundaryCategoryState = boundaryCategorySafety["state"]
                fusedLaneShadow = fusedLaneShadowAdvisor.update(
                    laneGuidance,
                    waypointSteering,
                    motion_allowed=(
                        boundaryCategorySafety["state"] != "STOP"
                    ),
                )
                continuousCorridor = continuousCorridorAdvisor.update(
                    laneGuidance,
                    pathCurvature,
                    waypointSteering,
                    dt,
                    motion_allowed=(
                        boundaryCategorySafety["state"] != "STOP"
                    ),
                )
                if controlMode.camera_steering:
                    boundarySteering = boundedHybridSteering.update(
                        waypointSteering,
                        laneGuidance,
                        dt,
                    )
                    delta = boundarySteering["final_steering_rad"]
                else:
                    delta = np.clip(
                        waypointSteering
                        + laneGuidance["applied_correction_rad"]
                        + fusedLaneShadow["applied_correction_rad"],
                        -steeringController.maxSteeringAngle,
                        steeringController.maxSteeringAngle,
                    )
                    boundarySteering = {
                        "state": "DISABLED",
                        "unconstrained_combined_rad": delta,
                        "constrained_waypoint_rad": waypointSteering,
                        "protective_correction_rad": (
                            laneGuidance["applied_correction_rad"]
                            + fusedLaneShadow["applied_correction_rad"]
                        ),
                        "constraint_adjustment_rad": 0.0,
                        "final_steering_rad": delta,
                    }
                if boundarySteering["state"] != lastBoundarySteeringState:
                    print(
                        f"[BOUNDARY STEERING] {lastBoundarySteeringState} -> "
                        f"{boundarySteering['state']}; "
                        f"waypoint={waypointSteering:.3f}, "
                        f"protective="
                        f"{boundarySteering['protective_correction_rad']:.3f}, "
                        f"final={delta:.3f}",
                        flush=True,
                    )
                    lastBoundarySteeringState = boundarySteering["state"]

                trackingLossSafety = trackingLossMonitor.update(
                    steeringController.cross_track_error,
                    steeringController.heading_error,
                    dt,
                )
                if trackingLossSafety["state"] != lastTrackingLossState:
                    print(
                        f"[TRACKING SAFETY] {lastTrackingLossState} -> "
                        f"{trackingLossSafety['state']}; "
                        f"crossTrack={steeringController.cross_track_error:.3f} m, "
                        f"headingError="
                        f"{np.degrees(steeringController.heading_error):.1f} deg, "
                        f"riskTime={trackingLossSafety['risk_duration_s']:.2f} s",
                        flush=True,
                    )
                    lastTrackingLossState = trackingLossSafety["state"]
                if controlMode.tracking_loss_stop:
                    vGain = min(vGain, trackingLossSafety["speed_gain"])
                if (
                    controlMode.tracking_loss_stop
                    and trackingLossSafety["state"] == "LOST"
                ):
                    delta = 0.0

                # In active mode this layer can only reduce speed authority.
                # Shadow mode preserves its state transitions in diagnostics
                # while exposing the unmodified waypoint-route behavior.
                if controlMode.boundary_speed_control:
                    vGain = min(vGain, boundaryCategorySafety["speed_gain"])

                #region : Speed controller update
                commandedSpeed = v_ref * vGain
                u = speedController.update(v, commandedSpeed, dt)
                #endregion
                if (
                    controlMode.localization_stop
                    and localizationSafety["state"] == "STALE"
                ):
                    # Do not steer from an EKF pose that is no longer being
                    # corrected by the positioning system.
                    delta = 0.0
                # A stopped vehicle gains nothing from holding a saturated
                # road-following angle. Neutralize the wheels after braking;
                # steering is recalculated normally when motion resumes.
                if commandedSpeed <= 1e-6 and abs(v) < 0.02:
                    delta = 0.0
            if t < startDelay:
                vGain = 0.0
            qcar.write(u, delta)            
            if enableSteeringControl and t >= startDelay:
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
                    waypoint_steering=waypointSteering,
                    intersection_phase=intersectionController.phase,
                    lane_safety_state=laneSafety["state"],
                    lane_safety_gain=laneSafety["speed_gain"],
                    lane_safety_confidence=laneSafety["confidence"],
                    lane_safety_offset=laneSafety["offset"],
                    lane_safety_age=laneSafety["age_s"],
                    lane_guidance_state=laneGuidance["state"],
                    lane_guidance_confidence=laneGuidance["confidence"],
                    lane_guidance_offset=laneGuidance[
                        "center_offset_normalized"
                    ],
                    lane_guidance_heading=laneGuidance["heading_error_rad"],
                    lane_guidance_candidate=laneGuidance[
                        "filtered_candidate_rad"
                    ],
                    lane_guidance_authority=laneGuidance["authority"],
                    lane_guidance_applied=laneGuidance[
                        "applied_correction_rad"
                    ],
                    lane_guidance_age=laneGuidance["age_s"],
                    lane_guidance_profile=laneGuidance[
                        "calibration_profile"
                    ],
                    lane_guidance_reason=laneGuidance[
                        "calibration_reason"
                    ],
                    lane_guidance_reference_x=laneGuidance[
                        "yellow_reference_x_px"
                    ],
                    lane_guidance_distance_mm=laneGuidance[
                        "estimated_center_to_yellow_mm"
                    ],
                    lane_guidance_physical_error=laneGuidance[
                        "physical_lateral_error_m"
                    ],
                    lane_guidance_path_curvature=laneGuidance[
                        "path_curvature_inv_m"
                    ],
                    lane_guidance_boundary_category=laneGuidance[
                        "boundary_category"
                    ],
                    lane_guidance_multirow_detected_count=laneGuidance[
                        "yellow_multirow_detected_count"
                    ],
                    lane_guidance_multirow_x_px=laneGuidance[
                        "yellow_multirow_x_px"
                    ],
                    boundary_safety_state=boundaryCategorySafety["state"],
                    boundary_safety_gain=boundaryCategorySafety["speed_gain"],
                    boundary_safety_risk_duration=boundaryCategorySafety[
                        "risk_duration_s"
                    ],
                    boundary_safety_inward_progress=boundaryCategorySafety[
                        "inward_progress_px"
                    ],
                    boundary_safety_recovery_from_peak=boundaryCategorySafety[
                        "recovery_from_peak_px"
                    ],
                    boundary_safety_recovery_streak=boundaryCategorySafety[
                        "recovery_streak"
                    ],
                    boundary_steering_state=boundarySteering["state"],
                    boundary_steering_unconstrained=boundarySteering[
                        "unconstrained_combined_rad"
                    ],
                    boundary_steering_constrained_waypoint=boundarySteering[
                        "constrained_waypoint_rad"
                    ],
                    boundary_steering_protective=boundarySteering[
                        "protective_correction_rad"
                    ],
                    boundary_steering_adjustment=boundarySteering[
                        "constraint_adjustment_rad"
                    ],
                    corridor_detected=laneGuidance["corridor_detected"],
                    corridor_confidence=laneGuidance["corridor_confidence"],
                    corridor_paired_rows=laneGuidance["corridor_paired_rows"],
                    corridor_offset=laneGuidance[
                        "corridor_offset_normalized"
                    ],
                    corridor_heading=laneGuidance["corridor_heading_rad"],
                    corridor_outer_source=laneGuidance[
                        "corridor_outer_source"
                    ],
                    quanser_yellow_detected=laneGuidance[
                        "quanser_yellow_detected"
                    ],
                    quanser_yellow_confidence=laneGuidance[
                        "quanser_yellow_confidence"
                    ],
                    quanser_yellow_slope=laneGuidance[
                        "quanser_yellow_slope"
                    ],
                    quanser_yellow_intercept=laneGuidance[
                        "quanser_yellow_intercept"
                    ],
                    quanser_shadow_steering=laneGuidance[
                        "quanser_shadow_steering_rad"
                    ],
                    corridor_trend_state=laneGuidance[
                        "corridor_trend_state"
                    ],
                    corridor_trend_offset_delta=laneGuidance[
                        "corridor_trend_offset_delta"
                    ],
                    corridor_trend_heading_delta=laneGuidance[
                        "corridor_trend_heading_delta"
                    ],
                    fused_lane_state=fusedLaneShadow["state"],
                    fused_lane_reason=fusedLaneShadow["reason"],
                    fused_lane_candidate=fusedLaneShadow[
                        "candidate_correction_rad"
                    ],
                    fused_lane_combined=fusedLaneShadow[
                        "combined_steering_rad"
                    ],
                    fused_lane_authority=fusedLaneShadow["authority"],
                    fused_lane_applied=fusedLaneShadow[
                        "applied_correction_rad"
                    ],
                    fused_lane_corridor_ready=fusedLaneShadow[
                        "corridor_ready"
                    ],
                    fused_lane_quanser_ready=fusedLaneShadow[
                        "quanser_ready"
                    ],
                    fused_lane_signals_agree=fusedLaneShadow[
                        "signals_agree"
                    ],
                    continuous_corridor_state=continuousCorridor["state"],
                    continuous_corridor_reason=continuousCorridor["reason"],
                    continuous_corridor_filtered_offset=continuousCorridor[
                        "filtered_offset_normalized"
                    ],
                    continuous_corridor_offset_rate=continuousCorridor[
                        "offset_rate_per_s"
                    ],
                    continuous_corridor_curvature_gain=continuousCorridor[
                        "curvature_gain"
                    ],
                    continuous_corridor_raw_candidate=continuousCorridor[
                        "raw_candidate_rad"
                    ],
                    continuous_corridor_bounded_candidate=continuousCorridor[
                        "bounded_candidate_rad"
                    ],
                    continuous_corridor_filtered_candidate=continuousCorridor[
                        "filtered_candidate_rad"
                    ],
                    continuous_corridor_combined=continuousCorridor[
                        "hypothetical_combined_rad"
                    ],
                    continuous_corridor_authority=continuousCorridor[
                        "authority"
                    ],
                    smooth_path_available=smoothPathShadow["available"],
                    smooth_path_segment_index=smoothPathShadow[
                        "segment_index"
                    ],
                    smooth_path_relative_segment=smoothPathShadow[
                        "relative_segment_offset"
                    ],
                    smooth_path_projection_x=smoothPathShadow[
                        "projection_x_m"
                    ],
                    smooth_path_projection_y=smoothPathShadow[
                        "projection_y_m"
                    ],
                    smooth_path_reference_heading=smoothPathShadow[
                        "reference_heading_rad"
                    ],
                    smooth_path_cross_track=smoothPathShadow[
                        "cross_track_error_m"
                    ],
                    smooth_path_heading_error=smoothPathShadow[
                        "heading_error_rad"
                    ],
                    smooth_path_control_speed=smoothPathShadow[
                        "control_speed_mps"
                    ],
                    smooth_path_feedback_steering=smoothPathShadow[
                        "feedback_steering_rad"
                    ],
                    smooth_path_steering=smoothPathShadow["steering_rad"],
                    smooth_path_authority=smoothPathShadow["authority"],
                    speed_control_state=speedController.state,
                    speed_integral_error=speedController.integral_error,
                    localization_safety_state=localizationSafety["state"],
                    localization_safety_gain=localizationSafety["speed_gain"],
                    localization_gps_age=localizationSafety["gps_age_s"],
                )
            # end = elapsed_time()
            if int(t * 2) != int(tp * 2):
                print(
                    f"[STATUS] speed={v:.2f} m/s, vGain={vGain:.2f}, "
                    f"throttle={u:.3f}, steering={delta:.3f}, "
                    f"trackingSafety={trackingLossSafety['state']}, "
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
                    f"feedback={steeringController.feedback_steering:.3f}, "
                    f"crossTrack={steeringController.cross_track_error:.3f} m, "
                    f"headingError="
                    f"{np.degrees(steeringController.heading_error):.1f} deg, "
                    f"laneSafety={laneSafety['state']}, "
                    f"laneGain={laneSafety['speed_gain']:.2f}, "
                    f"laneOffset={laneSafety['offset']:.3f}, "
                    f"laneAge={laneSafety['age_s']:.3f} s, "
                    f"hybrid={laneGuidance['state']}, "
                    f"hybridCandidate="
                    f"{laneGuidance['filtered_candidate_rad']:.3f}, "
                    f"hybridAuthority={laneGuidance['authority']:.2f}, "
                    f"hybridApplied="
                    f"{laneGuidance['applied_correction_rad']:.3f}, "
                    f"laneProfile={laneGuidance['calibration_profile']}, "
                    f"laneReason={laneGuidance['calibration_reason']}, "
                    f"lanePhysicalError="
                    f"{laneGuidance['physical_lateral_error_m']:.3f} m",
                    flush=True,
                )
            continue
        qcar.read_write_std(throttle= 0, steering= 0)
    localizationDiagnostics.close()
    trackingDiagnostics.close()
    intersectionController.close()
    if laneSafetyReceiver is not None:
        laneSafetyReceiver.close()
    if laneGuidanceReceiver is not None:
        laneGuidanceReceiver.close()

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
