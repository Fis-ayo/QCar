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
from tracking_diagnostics import TrackingDiagnostics
from lane_safety import LANE_SAFETY_PORT, LaneSafetyReceiver

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

args = parser.parse_args()
if not 0.0 <= args.tracking_offset_m <= 0.30:
    parser.error('--tracking_offset_m must be between 0.0 and 0.30 m')
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

        self.feedback_steering = wrap_to_pi(
            psi + np.arctan2(self.k*ect, speed)
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
    speedController = SpeedController(
        kp=K_p,
        ki=K_i
    )
    if enableSteeringControl:
        steeringController = SteeringController(
            waypoints=waypointSequence,
            k=K_stanley,
        )
        print(
            "[CONTROL] Steering mode: baseline; "
            f"tracking offset={args.tracking_offset_m:.3f} m",
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
            vGain = min(
                yoloGain,
                intersectionGain,
                laneSafety["speed_gain"],
            )
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

                #region : Speed controller update
                commandedSpeed = v_ref * vGain
                u = speedController.update(v, commandedSpeed, dt)
                #endregion
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
                    intersection_phase=intersectionController.phase,
                    lane_safety_state=laneSafety["state"],
                    lane_safety_gain=laneSafety["speed_gain"],
                    lane_safety_confidence=laneSafety["confidence"],
                    lane_safety_offset=laneSafety["offset"],
                    lane_safety_age=laneSafety["age_s"],
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
                    f"feedback={steeringController.feedback_steering:.3f}, "
                    f"crossTrack={steeringController.cross_track_error:.3f} m, "
                    f"headingError="
                    f"{np.degrees(steeringController.heading_error):.1f} deg, "
                    f"laneSafety={laneSafety['state']}, "
                    f"laneGain={laneSafety['speed_gain']:.2f}, "
                    f"laneOffset={laneSafety['offset']:.3f}, "
                    f"laneAge={laneSafety['age_s']:.3f} s",
                    flush=True,
                )
            continue
        qcar.read_write_std(throttle= 0, steering= 0)
    localizationDiagnostics.close()
    trackingDiagnostics.close()
    intersectionController.close()
    if laneSafetyReceiver is not None:
        laneSafetyReceiver.close()

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
