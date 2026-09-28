# Multi-QCar Autonomous Driving Project Status

**Last updated:** September 28, 2026

**Current milestone:** Candidate 5 stopped earlier but with both left wheels already beyond the divider; image offset is not yet calibrated to tire clearance

**Next milestone:** Stationary measured-clearance dataset across representative road geometries and both QCars

## Executive Summary

This project is developing a multi-vehicle autonomous-driving system on the Quanser Self-Driving Car Studio (SDCS) physical map. Two QCar2 vehicles can currently be launched from one local computer, follow predefined closed routes, communicate with a local intersection coordinator, stop at the central intersection, and receive mutually exclusive permission to enter it.

The following major capabilities have been demonstrated on the physical system:

- Both QCars can be launched and driven simultaneously.
- Each QCar has an explicit start node and a repeatable route that avoids unavailable portions of the physical map.
- The central conflict zone (CZ1) is represented in a shared road-network configuration.
- Both cars have stopped at CZ1 and one car has received priority while the other waited.
- The coordinator has prevented both vehicles from holding CZ1 simultaneously.
- Stale-network recovery has been improved after a temporary communication interruption caused a permanent standstill.
- Experimental lane-controller changes were rolled back after controlled tests. A general path-validation and conservative repair layer now removes unsafe generated-path spikes before driving.
- The rejected curvature and footprint-correction controller branches have now been removed from the active runtime after being preserved in `EXPERIMENTAL_REPORTS.md` and Git tag `pre-runtime-cleanup`.
- The repaired `.152` route has been physically validated through the node-16-to-node-18 transition that previously caused route departure.

The main unresolved issue is lateral path accuracy, especially through curves. Map waypoints and estimated pose currently control steering; the front CSI lane detector does not command steering. Its optional experimental safety layer can reduce speed or latch a stop, but Candidate 5 still stopped after both left wheels had crossed the divider, so its image threshold is not yet a valid physical-clearance boundary. A car that enters another car's lane can trigger the second car's YOLO vehicle detector, causing it to stop even if it has intersection priority. Lane tracking therefore needs to be quantitatively characterized and improved before expanding the reservation system to additional conflict zones.

The roadmap was revised after supervisor review. Phase 0 map definition and the Phase 1 CZ1 reservation foundation are implemented. The active work is now Phase 1A documentation and lane/trajectory reliability, followed by Phase 1B controlled two-vehicle interaction and Phase 1C repeated quantitative evaluation. Parking and multiple-zone expansion are deferred until these fundamentals pass their acceptance gates.

## Project Goal

The long-term goal is to simulate realistic multi-vehicle autonomous driving on the SDCS map. The system should eventually support:

- reliable lane and route following;
- collision-free interaction between multiple autonomous vehicles;
- stop signs, yielding, right-of-way, and first-arrival rules;
- roundabout behavior;
- traffic lights in selected areas;
- multiple simultaneous conflict zones;
- additional QCars without relying on permanent vehicle priority;
- perception-aware and communication-aware safety behavior; and
- later investigation of learning-based methods where they provide a measurable benefit.

Deep learning or reinforcement learning is not currently required for route following or basic intersection reservation. The present approach deliberately establishes deterministic localization, control, road rules, and safety behavior first.

## Physical and Network Setup

| Component | Address/identity | Current role |
|---|---|---|
| Local Windows computer | `192.168.0.153` | Launches both cars, runs the CZ1 coordinator and observer, stores coordinator logs |
| QCar `.152` | `192.168.0.152` | Starts at node 11; node configuration 1; current probing vehicle |
| QCar `.151` | `192.168.0.151` | Starts at node 0; node configuration 0 |
| Reservation service | UDP port `18750` | Local first-arrival coordinator for CZ1 |

Only one QCar was used for the existing LiDAR/GPS calibration and reference scan. Earlier work associated calibration and probing with a vehicle at `.151`; the current configuration assigns `CALIBRATE_IP` and `PROBING_IP` to `.152`. This history must be considered when comparing the localization performance of the two cars. Each physical vehicle may ultimately need its own verified localization and steering calibration.

The remote application directory is configured in `config.txt`. The launcher uploads all QCar-side Python and JSON files before each run, so the same current controller and road configuration are deployed to both vehicles.

## Current Routes

Routes are fixed and repeatable during the present testing stage. Random node selection was intentionally commented out because random routes previously used unavailable roads and created unpredictable conflicts.

| Vehicle | Start node | Route | CZ1 approach |
|---|---:|---|---|
| `.152` | 11 | `11 -> 12 -> 7 -> 14 -> 16 -> 18 -> 11` | North, associated with node 12 |
| `.151` | 0 | `0 -> 2 -> 4 -> 6 -> 0` | East, associated with node 6 |

These routes are stored in `qcar/road_network.json` and selected by `qcar/vehicle_control.py` using the explicit start-node assignments in `python/start.py`.

## Physical Map Availability

The road network distinguishes fully disabled nodes from physically present but currently inactive nodes.

- Disabled nodes: `10`, `22`, `23`
- Inactive nodes: `8`, `9`, `20`, `21`
- Nodes `8/9` and `20/21` are physically present, but their continuations lead toward unavailable road segments. They are retained in the map model for future use rather than being permanently removed.
- The current fixed routes do not use disabled or inactive nodes.
- Explicit disabled edges are recorded in `qcar/road_network.json`.

This representation allows unavailable segments to be re-enabled later without redesigning the route or intersection data model.

## Phase Roadmap and Status

The original roadmap moved directly from one-zone coordination to multiple
conflict zones. Supervisor feedback identified three prerequisites that must
now be completed first: reproducible source-code/architecture documentation,
reliable lane following, and an observable controlled two-vehicle interaction.
The revised phases below preserve completed work while inserting those gates.

### Phase 0 — Shared Road and Conflict-Zone Definition

**Status: Complete for CZ1 and the current routes. Future zones remain intentionally incomplete.**

Completed work:

- Created `qcar/road_network.json` as the shared machine-readable map and traffic-rules configuration.
- Added a schema version, map units, right-hand traffic designation, unavailable nodes, disabled edges, fixed routes, conflict zones, approaches, stop lines, exit areas, and priority policy.
- Created `qcar/road_rules.py` to load and validate the road-network data.
- Added validation that rejects malformed geometry, invalid node lists, open routes, and routes containing unavailable nodes.
- Defined CZ1 as a three-way all-way-stop intersection with the western approach retained but inactive.
- Defined approach polygons, stop lines, exit polygons, and permitted movements for the north, south, and east approaches.
- Recorded the proposed first-arrival policy, yield-to-the-right tie rule, and oldest-request fallback.
- Added `tools/visualization/plot_phase0_zones.py` and generated `docs/images/phase0_cz1_candidate.png` for visual verification of the configured geometry.
- Added placeholders for CZ2 and the roundabout without claiming that their geometry is ready.

Current CZ1 test movements are `12 -> 7` and `6 -> 0`. The west/node-9 approach remains represented but disabled.

### Phase 1 — One Conflict Zone With Automatic Reservation

**Status: Implemented and physically demonstrated; validation and refinement are still in progress.**

Completed software:

- Created `python/intersection_coordinator.py`, an automatic UDP reservation coordinator running on the local computer.
- Created `qcar/intersection_client.py`, containing the QCar-side communication client and intersection state machine.
- Implemented the states `CRUISING`, `APPROACHING`, `WAITING`, `GRANTED`, `OCCUPYING`, and `EXITED`.
- Implemented controlled approach speed, stop-line capture, minimum stop dwell, reservation waiting, conflict-zone occupancy, exit detection, and release.
- Implemented first-arrival/first-served reservation behavior.
- Added fairness behavior that can reserve the next opportunity for another vehicle already approaching CZ1, reducing repeated grants to the same car.
- Added fail-closed behavior: if the current owner becomes stale, CZ1 is locked rather than granted to another vehicle.
- Corrected a permanent-deadlock defect by clearing the safety lock when the same reservation owner resumes sending fresh heartbeats.
- Increased the coordinator stale timeout from 1.0 to 3.0 seconds and the QCar grant-validity timeout from 0.5 to 1.5 seconds to tolerate brief wireless interruptions.
- Added coordinator PID management and automatic startup/shutdown through the existing launch scripts.
- Added local coordinator logging to `intersection_coordinator.log`.
- Added QCar-side logs to `vehicle_control.log` and YOLO logs to `yolo_server.log` on each vehicle.
- Added `tests/test_phase1.py`. Six automated tests currently pass, including reservation ordering, release, fairness, stale-owner recovery, stop/grant/exit behavior, and communication loss while occupying the zone.

Physically observed behavior:

- Both cars have approached and stopped at CZ1.
- One vehicle has received permission and proceeded while the other waited.
- Both vehicles have completed repeated route cycles.
- A temporary communication interruption previously left both vehicles stationary. The log showed `.152` as the owner, a stale-owner safety lock, and `.151` waiting. Recovery logic was corrected and locally tested.
- The latest coordinator log confirms that a stale `.152` owner later restored communication and the safety lock was cleared without granting CZ1 to `.151` simultaneously.

Remaining Phase 1 validation:

- Confirm over extended physical runs that the waiting vehicle proceeds promptly after the owner exits.
- Confirm fairness over many consecutive laps.
- Measure packet loss and heartbeat interruptions on both QCars.
- Verify all configured stop and exit polygons against physical trajectories.
- Ensure lane deviation does not place a yielding car in the priority vehicle's path.
- Resolve reservation-aware interaction with YOLO without weakening collision safety.

This implementation is the coordination foundation for Phases 1B and 1C. It
is not considered fully validated solely because both vehicles have moved
through CZ1 during an uncontrolled-duration run.

### Phase 1A — Documentation and Lane/Trajectory Reliability

**Status: Active and highest priority for the next report.**

Required documentation deliverables:

- provide the relevant source code in the report appendix or a clearly
  accessible repository;
- document installation, configuration, deployment, execution, stopping, and
  log collection;
- create a system architecture diagram covering route planning, localization,
  path tracking, YOLO safety, CZ1 coordination, communication recovery, and
  actuator commands; and
- explain the responsibilities and interfaces of the principal source files.

Documentation foundation completed September 25, 2026:

- `README.md` now provides prerequisites, configuration, launch, shutdown,
  logging, safety, and local-verification instructions;
- `docs/SYSTEM_ARCHITECTURE.md` contains deployment, control/data-flow, CZ1
  sequence, communication-recovery, and safety-authority diagrams; and
- `docs/SOURCE_CODE_GUIDE.md` maps the requested report components to their
  implementation and test files.

Repository publication and selection of source excerpts for the report appendix
remain pending; neither requires changing vehicle behavior.

Required lane/trajectory work:

- establish repeated single-vehicle baseline runs on both fixed routes;
- validate whether generated and offset reference trajectories follow the
  physical centers of their designated lanes;
- distinguish reference-path, localization, controller, steering/actuator,
  and vehicle-footprint causes before changing control gains;
- quantify mean absolute, RMS, maximum, and 95th-percentile lateral error,
  with straight and curved sections reported separately;
- record lane-boundary violations, steering saturation, and successful-lap
  rate; and
- compare the accepted improved method against the unchanged baseline.

Camera integration is now the leading solution direction, but remains staged
and evidence-gated. Global waypoint navigation must continue to select the
route and intersection movement. Candidate 3 saved frames confirmed that the
front CSI detector tracked the physical yellow divider and issued a warning
`2.86 s` before shutdown while map-reference error remained small. The first
proposed control authority is a bounded safety slowdown/stop; bounded steering
fusion remains a later step after repeated validation.

The current steering controller does not use camera lane measurements. It follows a path generated by `SDCSRoadMap.generate_path()` using a Stanley-style steering controller and an EKF pose estimate derived from LiDAR/GPS, gyroscope, motor tachometer, and steering input. A front CSI yellow-line detector is available for measurement-only diagnostics and cannot modify steering commands.

Observed problem:

- Vehicles do not consistently maintain their lanes, particularly through curves.
- If the yielding vehicle crosses into the priority vehicle's lane, the priority vehicle's YOLO car detector can stop it. This can create a practical standstill even though the reservation coordinator has selected the correct owner.

Current implementation:

- Restored the original SDCS Stanley gain of `0.7`.
- Restored projection-based one-segment waypoint advancement.
- Restored the original Stanley speed term and constant nominal route speed of `0.25 m/s`.
- Retained non-controlling telemetry for estimated position, waypoint index, distance to the next waypoint, cross-track error, and heading error.
- Removed the experimental curve-speed, capture-radius, low-speed-softening, and tracking-fault behavior after physical runs showed waypoint progression regressions.

Latest `.152` physical result after path repair:

- The car followed the intended route from node 16 toward node 18.
- Maximum logged cross-track error fell from `1.084 m` to `0.086 m`.
- Maximum logged heading error fell from `147.9 degrees` to `38.9 degrees`.
- The former waypoint-804 lock and approximately 180-degree heading reversal did not recur.
- Steering still reached its `0.524 rad` limit in tight portions of the roundabout, so lane centering is improved but not considered complete.

Required physical validation:

1. Repeat `.152` runs as needed to establish mean, maximum, and repeatability of lateral error rather than relying on one successful run.
2. Run `.151` independently for at least one complete lap using the same diagnostics.
3. Compare cross-track error, heading error, steering saturation, and physical lane position at corresponding curves.
4. Note whether either vehicle consistently drifts in one direction.
5. If only one car shows the error, investigate its calibration, pose estimate, steering-center offset, tire/mechanical alignment, and steering scale.
6. Only after individual behavior is characterized, repeat the two-car CZ1 experiment.

The current physical observations distinguish two symptoms: `.152` follows
the intended route but is not consistently centered through curves, whereas
`.151` has moved toward the node-5 lane near the node-2-to-node-4 curve instead
of passing node 4. Neither symptom has yet been assigned a root cause.

The first instrumented `.151` run shows that its controller reference remained
on the intended node-4 outer path and waypoint progress remained monotonic. Its
tracking point instead stayed roughly `0.05–0.08 m` inside the reference on the
eastern curve. Localization innovations were small near the departure and no
steering saturation occurred in that curve. This rules against an observed
wrong-node/path switch in that run, but a repeat run and `.152` comparison are
needed before separating shared control/map alignment from vehicle-specific
calibration or mechanical effects.

A second `.151` run reproduced the lane crossing in the same lower-right
curve. The intended reference and monotonic route progress were again correct;
median eastern-curve error was approximately `0.055 m`, with a `0.090 m`
maximum and no curve-region steering saturation. This makes the inward
tracking symptom repeatable rather than incidental. An independent `.152` run
with the same diagnostics is now the next comparison step.

The subsequent `.152` comparison shows the same signed inward bias while its
reference path, waypoint progress, and localization remain valid. Median
absolute cross-track error was approximately `0.039 m` from node 11 toward
node 7 and `0.042 m` from node 7 toward node 14, with maxima near `0.08–0.09
m`; more than 94% of samples in those regions had the same negative sign.
Steering did not saturate on those ordinary curves. In the roundabout,
however, steering saturated in approximately 25% of moving samples and heading
error reached approximately `39 degrees`. The evidence now supports a shared
curved-path tracking limitation, with insufficient steering authority or
reference anticipation in the tightest curve, rather than separate node-level
route failures.

The next measurement stage is implemented. Tracking diagnostics now calculate
local curvature, the strongest curvature within the next `0.50 m`, and the
corresponding bicycle-model steering demand using the QCar's documented
`0.256 m` wheelbase. It also records whether predicted demand exceeds the
current `30 degree` steering limit. This does not yet change vehicle behavior;
it provides the evidence required to design bounded curvature feedforward and
curve-speed scheduling for both vehicles and all routes.

The first curvature-instrumented `.152` run found maximum predicted geometric
steering demand of approximately `22.4 degrees`, below the existing `30
degree` command limit. Nevertheless, actual steering saturated during about
`24.5%` of moving roundabout samples. This shows that curvature alone is not
beyond the vehicle's nominal steering range; the current error feedback grows
large enough to cause saturation. Feedforward therefore must not be naively
added to the existing saturated command. The matching `.151` dataset is still
needed before defining the bounded combined controller.

The matching `.151` run completed the comparison. Its lower-right curve had
approximately `0.049 m` median and `0.082 m` maximum absolute error, with
about `98%` of samples biased inward. Upcoming geometric steering demand was
approximately `16 degrees` while the baseline controller averaged only about
`10.4 degrees` across the analyzed transition and did not saturate. Stable
localization and correct route progress again rule against a node-specific
path or pose failure. Curvature-demand characterization is therefore complete;
the next step is a switchable, conservative A/B implementation of filtered
curvature feedforward plus curve-speed scheduling while retaining bounded
Stanley feedback.

That A/B implementation is now available. Curvature-aware mode previews `0.20
m` for smoothly filtered steering feedforward, uses a `0.50 m` horizon for
curve-speed reduction from `0.25 m/s` toward a `0.18 m/s` minimum, rebalances
heading feedback, and preserves the existing steering and safety limits. The
launcher enables the mode per vehicle through `curvature_control`; setting it
to `False` restores the baseline. Physical acceptance remains pending, starting
with `.151` alone.

The first `.151` A/B result did not pass acceptance. Median curve error
increased from approximately `0.049 m` to `0.058 m`, and maximum error from
`0.082 m` to `0.091 m`; the car again crossed toward the node-5 lane. Although
feedforward was active, the simultaneous global reduction in heading feedback
left total steering only slightly above baseline. Both launcher settings are
therefore restored to baseline (`curvature_control: False`). The experimental
implementation remains switchable in source for redesign, but it should not
be used on `.152` or in two-car operation in its current form.

Candidate 2 now preserves stronger heading feedback on moderate curves and
reduces it progressively only near tight-roundabout curvature, while increasing
filtered feedforward modestly. It is enabled only for `.151`; `.152` remains
on baseline. The next physical acceptance test must show both numerical error
improvement and no crossing into the node-5 lane before the candidate is
considered for `.152`.

Candidate 2 failed physically: node-2-to-node-4 median error increased to
approximately `0.076 m` and maximum to `0.103 m`, with even larger error in
the preceding bottom curve. The result showed that positive curvature
feedforward was increasing inward corner cutting. Candidate 3 therefore
removes feedforward, preserves full straight-line heading gain, and reduces
heading gain only with increasing curvature so cross-track feedback can push
the car away from the inside divider. It is enabled only for `.151` and logs
as `curvature_feedback_v3`.

Candidate 3's first `.151` run improved node-2-to-node-4 median error from
approximately `0.049 m` to `0.020 m` and its 95th percentile from `0.076 m` to
`0.037 m`. The earlier bottom-curve median improved from `0.070 m` to `0.034
m`; its apparent `0.125 m` maximum occurred immediately after startup, while
the later curve maximum was approximately `0.086 m`. Localization was stable
and route progress was normal. Physical acceptance is still pending because
the submitted run did not state whether the car crossed either lane boundary.

Physical follow-up confirmed that `.151` still crossed into the node-5 lane,
although node-0-to-node-2 boundary behavior appeared better. Candidate 3 is
therefore disabled in the launcher. The mismatch between improved
tracking-point error and continued body encroachment shifts the investigation
to vehicle geometry: the controller tracks a point `0.20 m` ahead of the EKF
pose, the physical configuration documents a `0.256 m` wheelbase, and the EKF
model uses `0.20 m`. Map-to-physical lane alignment and the vehicle's
center/front/rear/swept footprint must be measured before another controller
gain experiment.

That measurement instrumentation is now implemented. The tracking log records
estimated body-center, front/rear axle, and four wheel-center locations and
their signed errors relative to the active path. It uses the documented
`+/-0.130 m` axle offsets and `0.086 m` half-track, while explicitly labeling
the unverified assumption that the EKF pose is the body center. No controller
behavior is changed, and both launcher entries remain on baseline.

The first baseline footprint run on `.151` is now complete. On the
node-2-to-node-4 curve, the forward steering reference had a median absolute
error of about `0.050 m`, while the assumed body center was about `0.120 m`,
the rear axle about `0.166 m`, and the inside wheel about `0.248 m` from the
same path. Localization remained stable and produced no event captures. This
supports a general curved-path geometry/reference-point mismatch: the point
used by the controller can look acceptable while the rear of the physical car
cuts inward. It does not yet prove the exact correction because the EKF pose
origin must be verified.

The follow-up `0.13 m` front-axle-offset test improved node-2-to-node-4 median
absolute errors: tracking/front axle from approximately `0.050 m` to `0.033
m`, assumed body center from `0.120 m` to `0.068 m`, rear axle from `0.166 m`
to `0.102 m`, and inside wheel from `0.248 m` to `0.189 m`. The gradual
node-0-to-node-2 encroachment was no longer observed, but `.151` still entered
the node-5 lane on the tighter curve. Localization remained stable. The
geometry change is therefore useful but insufficient; the system currently
has no direct measurement of its position between the physical lane lines.

A measurement-only front-CSI subsystem is now implemented and enabled only on
`.151` for initial validation. It segments initial yellow/white lane-marking
candidates, estimates left/right boundaries, lane-center offset, heading, and
confidence, and saves `lane_diagnostics.csv`, `lane_camera.log`, and annotated
frames under `lane_frames/`. It cannot command the vehicle. YOLO remains on
the separate RealSense RGB/depth stream, and `.152` remains unchanged.

The first 3:27 PM physical image set confirmed that the CSI view and ROI are
usable, but invalidated the initial two-color lane-center output. The yellow
line is segmented visibly, while the white mask often selects the large bright
curb/platform instead of a painted boundary. The apparent `85.5%` detection
rate is therefore dominated by false positives and must not be used for
steering. Detection also dropped to `44.5%` around node 2 to node 4.
Localization stayed stable. The next vision revision will retain measurement-
only operation, save raw plus annotated frames, use the documented QCar2
yellow-line method as the interpretable baseline, and permit an offline
LaneNet comparison on identical evidence.

That revision is now implemented. It fits a curved yellow-line component and
logs lateral position, heading, curvature, coverage, fit error, temporal jump,
and confidence. Filled regions and discontinuous/jumping candidates are
penalized or rejected. Raw and annotated images are saved separately, and an
offline reprocessing tool permits repeatable detector comparisons without
additional driving. Camera output remains completely disconnected from vehicle
control.

The second `.151` camera run confirmed that the fitted feature usually follows
the visible yellow marking, with median fit error near `5 px`. Yellow detection
was available for `55.4%` of synchronized start/node-0-to-node-2 samples and
`86.9%` around node 2 to node 4. However, tight curves can cause the fitted
near point to extrapolate outside the image while retaining high confidence,
and yellow visibility was lost after waypoint 400. The current confidence is
therefore a feature-fit score, not yet a safe lane-control confidence. The
saved raw frames allow the next correction to be developed offline.

The observed-span correction has now been implemented and reprocessed against
the 3:58 PM raw images. It accepted 30 of 34 snapshots and kept accepted near
positions inside the image. The installed CSI API exposes no intrinsic or
distortion calibration, so the software now records
`perspective_calibrated=0` and continues to prohibit camera steering. Physical
camera calibration and ground-plane mapping remain possible future options,
but are deferred pending supervisor feedback; inventing camera parameters from
the route images would not be defensible.

`qcar/tracking_diagnostics.py` now records the estimated vehicle pose, forward
tracking point, active reference segment, projected reference, route progress,
tracking errors, speed, actuator commands, steering saturation, and
intersection phase at 20 Hz. This measurement-only recorder is shared by both
vehicles and all routes. `tools/analysis/analyze_tracking.py` provides common summary
metrics and plots. The resulting evidence will distinguish reference-path,
localization, waypoint-progress, controller/actuator, and vehicle-specific
causes before control parameters are changed.

No unmeasured per-car steering offset has been added. Such a correction should be based on physical-run data rather than assumption.

### Localization Research and Instrumentation

**Status: Measurement instrumentation implemented; localization remains monitored but was not the cause of the latest node-16 route departure.**

One earlier baseline log appeared to contain six estimated-position changes greater than `0.35 m` between half-second status samples, including changes of approximately `1.1–1.2 m`. That observation motivated raw-pose and EKF instrumentation, but it was treated as a hypothesis rather than a confirmed localization failure.

Two later instrumented runs did not reproduce a raw localization discontinuity. In the most recent successful run, median position innovation was approximately `0.020 m`, maximum post-startup innovation was approximately `0.085 m`, maximum post-startup heading innovation was approximately `1.9 degrees`, and no raw-position jump exceeded `0.2 m`. The localization-event directory was correctly empty. Localization is therefore still monitored, but the proven cause of the node-16-to-node-18 departure was malformed generated-path geometry.

Measurement-only instrumentation now records:

- raw scan-matched `QCarGPS` position and heading;
- EKF pose immediately before and after each measurement correction;
- position and heading innovation;
- motor speed, steering command, and gyroscope yaw rate;
- LiDAR valid-point count and minimum, median, and maximum range;
- GPS measurement age in the ordinary vehicle status log; and
- the most recent full LiDAR scan history when a pose innovation exceeds `0.35 m` or `45 degrees`.

This instrumentation does not reject measurements and does not change steering, speed, waypoint progression, or EKF behavior.

Diagnostic outputs on each QCar:

- `localization_diagnostics.csv`: one row per new scan-matched pose;
- `localization_events/event_*.npz`: compressed raw LiDAR histories and vehicle state surrounding detected outlier events; and
- `vehicle_control.log`: human-readable pose, innovation, scan-quality, and path-tracking status.

Research sequence to preserve:

1. Correlate physical/video behavior with raw scan-matched pose and EKF discontinuities.
2. Determine whether the jump originates in raw scan matching or elsewhere in EKF/state handling.
3. Inspect LiDAR scan completeness and raw scans around detected events.
4. Verify per-vehicle calibration files, initial calibration pose, sensor mounting, and reference-scan provenance.
5. Evaluate innovation gating or physically plausible measurement validation without first changing controller behavior.
6. Investigate pose-constrained scan matching and available match-quality/confidence information.
7. After localization is trustworthy, redesign waypoint progress using a bounded forward search or route arc length.
8. Compare Stanley and Pure Pursuit using the same validated localization data.
9. Tune speed by curvature and characterize per-car steering alignment.
10. Evaluate camera lane detection as an independent lateral correction source and fuse it with map localization.

### Milestone: Route-Integrity Validation and General Waypoint Repair

**Status: Implemented, locally tested, and physically validated on `.152`; `.151` still requires independent physical validation.**

Inspection of the latest `.152` run separated localization behavior from a deterministic route-geometry defect. Raw scan-matched pose and EKF innovations remained small, but the generated route contained a paired heading reversal immediately before node 16. Segment direction changed by `-179.57 degrees` and then `+179.57 degrees`, causing the baseline Stanley controller to select a backward-pointing reference, saturate steering, and depart from the route toward node 18.

A general `PathManager` now runs between `SDCSRoadMap.generate_path()` and the steering controller. It:

- validates coordinate shape and finite values;
- rejects zero-length segments and excessive waypoint gaps;
- detects near-180-degree heading reversals;
- checks closure for closed node routes;
- verifies that every requested route node remains represented;
- identifies paired opposite reversals as a conservative spike pattern;
- protects exact route-node anchors;
- removes only a non-anchor spike when the resulting path revalidates; and
- prevents vehicle startup if unsafe geometry remains.

Verified results:

- Route `11 -> 12 -> 7 -> 14 -> 16 -> 18 -> 11` changed from 992 to 991 waypoints.
- The non-anchor point at original waypoint 804 was removed while node 16 was preserved.
- Maximum adjacent heading change fell from `179.57 degrees` to `1.03 degrees`.
- Maximum waypoint gap remains `0.0182 m`.
- Route `0 -> 2 -> 4 -> 6 -> 0` requires no repair and remains unchanged.
- The Stanley gain, nominal speed, localization correction, intersection behavior, and baseline waypoint progression were not changed.

The raw and prepared routes can be reviewed in `docs/images/validated_routes.png`, generated by `tools/visualization/plot_validated_routes.py`.

Physical before-and-after evidence:

| Measurement | Before repair | After repair |
|---|---:|---:|
| Generated waypoints | 992 | 991 |
| Maximum adjacent heading change | `179.57 degrees` | `1.03 degrees` |
| Maximum logged cross-track error | `1.084 m` | `0.086 m` |
| Maximum logged heading error | `147.9 degrees` | `38.9 degrees` |
| Progress from node 16 toward node 18 | Failed | Successful |
| Raw localization event during comparison run | None | None |

Research significance:

- The observed actuator behavior initially resembled a steering, gain, or localization problem, but the controller was responding to an invalid reference path.
- The investigation established the principle that planning and reference data must be validated before controller tuning.
- The solution is route-independent and fail-closed rather than a special case for node 16.
- The remaining lateral error can now be studied without the paired-reversal defect contaminating controller measurements.

Phase 1A exit criteria:

- the reference trajectory is visually and numerically validated against the
  intended physical lane center on the tested routes;
- a repeatable baseline and after-improvement dataset exists;
- both vehicles complete their individual routes without entering an opposing
  or adjacent designated lane; and
- the report explains why the accepted change improves lane tracking.

### Phase 1B — Controlled Two-Vehicle Interaction

**Status: Coordinator available; controlled demonstration not yet completed.**

The next two-car demonstration must be intentionally staged rather than left
to coincidental arrival times. Both vehicles will approach CZ1 at approximately
the same time. Logs and video must clearly show request, stop, grant, occupancy,
release, and resumption by the waiting vehicle.

A second scenario will place one vehicle behind another. The following vehicle
must respond safely when the lead vehicle stops and resume only when the path
is clear. This scenario requires a defined lead-vehicle response and must not
be represented as complete merely because YOLO happened to stop a car.

Phase 1B exit criteria:

- no simultaneous CZ1 ownership or occupancy;
- the non-granted vehicle stops before entering the conflict zone;
- the waiting vehicle resumes after release without unreasonable delay;
- the lead/follow scenario produces a repeatable, explainable response; and
- synchronized video, vehicle logs, and coordinator logs show the interaction.

### Phase 1C — Repeated Quantitative Evaluation

**Status: Instrumentation partly available; formal repeated dataset not started.**

The next report must include repeated trials and before/after comparisons for:

- lane-tracking error;
- intersection waiting time;
- successful crossing rate;
- communication recovery time; and
- minimum separation distance between vehicles.

Each formal experiment must identify the vehicle, route, software/configuration
version, trial count, success definition, safety intervention, and preserved
evidence. Phase 1C is complete only when the reported results can be reproduced
from archived logs and configurations.

### Phase 2 — Autonomous Parking

**Status: Deferred until Phases 1A–1C are reliable.**

Begin with one QCar leaving its designated lane and following a validated
planned trajectory into one measured parking space. Before implementation,
confirm that the physical map provides sufficient approach, maneuvering, and
stopping dimensions. A later extension may coordinate two vehicles requesting
parking spaces and prevent conflicting parking movements.

### Phase 3 — Multiple Conflict Zones

**Status: Not started.**

This phase extends the reservation mechanism beyond CZ1. It begins only after
the Phase 1 lane, interaction, and evaluation gates are satisfied. It may be
scheduled before the two-vehicle extension of parking, depending on the next
research objective.

Candidate areas:

- `CZ2_EAST_MID`, currently proposed as an all-way stop and potentially a future priority/yield intersection;
- additional marked central/eastern conflict areas from the annotated map; and
- the roundabout, which requires entry-specific yield behavior rather than ordinary all-way-stop logic.

Phase 3 will require:

- confirmed polygons, approaches, stop lines, exits, and permitted movements for every zone;
- support for one vehicle approaching or occupying different zones in sequence;
- independent reservation state per zone;
- deadlock avoidance when vehicles request multiple nearby zones;
- routing-aware identification of the next relevant zone;
- clear rules for release and recovery in every zone; and
- expanded automated and physical tests.

### Later Phases

Planned capabilities, not yet implemented:

- yield and priority-road intersections;
- stop-sign recognition integrated with map rules;
- traffic lights and signal-phase communication;
- formal roundabout entry and circulating-traffic priority;
- a third QCar;
- richer vehicle-to-infrastructure or vehicle-to-vehicle messages;
- dynamic routing around unavailable roads;
- perception fusion and obstacle classification;
- possible learning-based decision or prediction components after deterministic baselines are established.

## Software Architecture

The current control flow is:

```text
run_all_cars.bat
    -> python/start.py on the local computer
        -> starts intersection_coordinator.py locally
        -> uploads qcar/*.py and qcar/*.json to both QCars
        -> starts yolo_server.py on each QCar
        -> starts vehicle_control.py on each QCar

vehicle_control.py
    -> loads and validates road_network.json
    -> creates the fixed SDCS waypoint route
    -> validates and conservatively repairs generated path geometry
    -> estimates pose with QCarEKF and QCarGPS
    -> calculates baseline Stanley steering at the configured nominal speed
    -> applies YOLO and intersection speed restrictions
    -> exchanges reservation state with the local coordinator
    -> writes throttle and steering commands to the QCar
```

The coordinator is automatic, not manually operated. The operator launches and stops the experiment; reservation requests, priority selection, locking, and releases occur from vehicle state messages.

## Safety Behavior

Current safety decisions include:

- A car remains stopped if its initial measured pose differs from its declared start node by more than `0.5 m` or its heading differs by more than 60 degrees.
- Startup waits for a plausible LiDAR/GPS fix and rejects distant initial fixes for a limited period.
- A QCar cannot run normal driving without a coordinator address and car identity.
- A QCar refuses to start if its generated path remains invalid after conservative preparation.
- A waiting car cannot enter CZ1 without a fresh grant.
- If communication is lost before entry, a granted vehicle returns to waiting.
- If communication is lost after the vehicle is already occupying the conflict zone, it continues clearing the zone rather than stopping inside it.
- A stale reservation owner locks the zone against every other car.
- A returning heartbeat restores only the existing owner's grant; it does not create a second owner.
- The speed controller clears accumulated integral throttle for a stop request, reducing stop-line creep.
- YOLO obstacle behavior remains enabled. It has not been bypassed to solve reservation problems.

The system remains experimental and requires supervision and accessible emergency stopping during every physical run.

## Important Challenges and Resolutions

### Cars initially did not move

Causes encountered included mismatched start poses, missing or inconsistent route nodes, reliance on map segments unavailable on the physical layout, and HAL-version incompatibility involving `SDCSRoadMap.initial_check()`.

Resolution:

- added explicit per-car start nodes and deterministic routes;
- added startup pose validation and clearer diagnostics;
- bypassed unnecessary route recovery when a declared physical start node is supplied;
- retained an `initial_check()` compatibility guard for cases without an explicit start node; and
- deployed the same required scripts and JSON configuration to both cars.

### Logs were missing or difficult to locate

Resolution:

- remote processes now use `nohup` and redirect output to `vehicle_control.log` and `yolo_server.log` in the configured remote directory;
- the local coordinator writes `intersection_coordinator.log` in the project root; and
- the launcher prints the coordinator log location.

### Random routes used unsuitable map segments

Resolution:

- randomized selection was commented out rather than deleted;
- road availability is recorded separately from routes; and
- current experiments use two fixed, validated, closed routes.

### Cars collided after repeated laps

Cause:

- simultaneous route following did not originally include explicit conflict-zone ownership, and route timing eventually brought both vehicles into the same area.

Resolution:

- introduced CZ1 geometry, stop behavior, automatic first-arrival reservation, occupancy/release state, and fairness handling.

### Waiting car remained stopped while the first car returned

Contributing issues included reservation-release timing, fairness between repeated arrivals, and later a stale-owner lock that did not recover.

Resolution:

- added next-car preference when another car is already approaching;
- added explicit exit polygons and release behavior; and
- corrected stale-owner heartbeat recovery while retaining fail-closed safety.

### Vehicles leave their lanes on curves

Likely contributors include fixed-speed cornering, path-segment tracking, Stanley gain, localization error, transferring calibration between physical vehicles, and possible per-car steering alignment differences.

Investigation and current resolution:

- added quantitative path, localization, and control telemetry;
- tested several waypoint, speed, and gain changes, then rolled them back after they introduced regressions;
- discovered and repaired a general paired-reversal defect in generated route geometry;
- physically verified that `.152` now continues from node 16 toward node 18; and
- retained the original Stanley gain and speed so the remaining lane-centering problem can be characterized cleanly.

Route departure at node 16 is resolved for `.152`. General lane centering is not yet considered resolved.

## Running and Stopping the System

Before running:

1. Confirm both QCars are powered, connected, and placed at their assigned start nodes.
2. Confirm `.152` is at node 11 and `.151` is at node 0 with the expected orientations.
3. Check `config.txt` for the local and QCar IP addresses.
4. Ensure no earlier controller or coordinator instance is still running.
5. Keep the physical map clear and maintain access to emergency stopping.

Start from the project root:

```bat
run_all_cars.bat
```

Keep the launcher window open. `python/start.py` owns the local coordinator process and prompts before exiting.

Stop the system:

```bat
stop_all_cars.bat
```

Useful logs:

| Log | Location | Purpose |
|---|---|---|
| Coordinator | Local project root: `intersection_coordinator.log` | Queue, grant, release, stale-owner, and recovery events |
| Vehicle controller | Remote application directory on each QCar: `vehicle_control.log` | Pose, state transitions, speed, throttle, steering, tracking error |
| YOLO server | Remote application directory on each QCar: `yolo_server.log` | Perception-server startup and runtime failures |
| Localization CSV | Remote application directory on each QCar: `localization_diagnostics.csv` | Raw scan-matched pose, EKF before/after correction, innovations, motion and LiDAR summaries |
| Localization events | Remote application directory on each QCar: `localization_events/event_*.npz` | Full recent LiDAR scans captured around large pose innovations |

Run local validation:

```powershell
python qcar/road_rules.py
python tests/test_phase1.py
python tests/test_path_manager.py
python tests/test_localization_diagnostics.py
python -m py_compile qcar/vehicle_control.py qcar/path_manager.py qcar/localization_diagnostics.py qcar/intersection_client.py python/intersection_coordinator.py
```

At the time of this update, the full local discovery suite passes all 23 automated tests.

## Key Files

| File | Responsibility |
|---|---|
| `config.txt` | Local/QCar addresses, probing assignment, remote path, observer dimensions |
| `run_all_cars.bat` | Reads configuration and invokes the local launcher |
| `stop_all_cars.bat` | Stops QCar applications and local services |
| `README.md` | Prerequisites, configuration, operation, safety, logs, and validation instructions |
| `docs/SYSTEM_ARCHITECTURE.md` | Deployment, data-flow, CZ1 sequence, and safety-authority diagrams |
| `docs/SOURCE_CODE_GUIDE.md` | Report-oriented implementation and automated-test inventory |
| `docs/LANE_TRACKING_BASELINE_PROTOCOL.md` | Frozen baseline procedure, route sections, metrics, evidence, and completion criteria |
| `experiments/templates/trial_observations.csv` | Standard physical lane-event observation record |
| `EXPERIMENTAL_REPORTS.md` | Chronological experimental questions, configurations, evidence, failures, and conclusions |
| `python/start.py` | Starts coordinator, deploys files, and launches each QCar with explicit settings |
| `python/stop.py` | Stops remote Python processes and the local coordinator |
| `python/intersection_coordinator.py` | Automatic CZ1 reservation and safety locking |
| `tests/test_phase1.py` | Local coordinator and vehicle-state-machine tests |
| `tools/visualization/plot_phase0_zones.py` | Visualization of configured road and conflict-zone geometry |
| `tools/visualization/plot_validated_routes.py` | Raw-versus-prepared route visualization |
| `tools/visualization/plot_reference_validation.py` | Repeated-trial reference, body, wheel, and map-derived lane-midpoint overlay |
| `tools/analysis/analyze_footprint_envelope.py` | Two-sided measured-boundary fit and wheel-envelope violation analysis |
| `tools/analysis/analyze_camera_tracking_alignment.py` | Time-aligns CSI features with modeled two-sided wheel clearance |
| `tests/test_path_manager.py` | General path validation, repair, anchor preservation, and fail-closed tests |
| `tests/test_localization_diagnostics.py` | Localization CSV and raw LiDAR event-capture test |
| `qcar/road_network.json` | Shared map availability, routes, zones, approaches, and traffic rules |
| `qcar/road_rules.py` | Road-network loading and validation |
| `qcar/path_manager.py` | General generated-path validation and conservative spike repair |
| `qcar/localization_diagnostics.py` | Raw pose/EKF diagnostics and LiDAR event capture |
| `qcar/intersection_client.py` | QCar reservation client and stop/intersection state machine |
| `qcar/vehicle_control.py` | Localization, speed control, steering, perception response, and reservation integration |
| `qcar/yolo_server.py` | QCar-side perception publisher |

## Immediate Next Actions

1. Preserve Candidate 3 as evidence that physical boundary approach can occur while map-reference error remains small.
2. Define the camera safety-slowdown/stop thresholds, confidence requirements, debounce, fail-safe behavior, and acceptance criteria offline.
3. Implement camera intervention behind an explicit launcher flag, with baseline behavior remaining the default and every intervention logged.
4. Conduct supervised `.151` A/B trials measuring warning lead time, intervention count, boundary violations, successful laps, and false stops.
5. Repeat the accepted method on `.152` to show that the solution is route- and vehicle-general rather than node-specific.
6. Update the supervisor report using `docs/NEXT_SUPERVISOR_REPORT.txt`, including source-code references, architecture, limitations, and before/after results.
7. Stage the controlled same-time CZ1 interaction and lead/follow scenarios after individual lane reliability is acceptable.
8. Calculate lane error, waiting time, crossing success, recovery time, and minimum separation for the next report.
9. Begin parking or CZ2/multiple-zone implementation only after the Phase 1A–1C acceptance gates are met.

## Evidence Needed for a Future Paper or Presentation

Future runs should record consistent quantitative evidence rather than relying only on observation:

- route and vehicle identity;
- number of completed laps;
- mean and maximum cross-track error;
- mean and maximum heading error;
- stop-line position error;
- intersection waiting time per vehicle;
- reservation grants and releases;
- conflict-zone occupancy duration;
- number and duration of communication interruptions;
- collision, near-collision, emergency-stop, and YOLO-stop events;
- throughput with one, two, and later three vehicles; and
- before/after comparisons for each controller change.

Videos should be time-aligned with QCar and coordinator logs where possible. Configuration files and code versions should be archived for every formal experiment.

## Known Limitations

- Only CZ1 has complete, validated software geometry.
- The western CZ1 approach is inactive.
- CZ2 and roundabout geometry are placeholders.
- The system coordinates one zone and two fixed routes.
- Current tie-breaking data exists in the road configuration, but simultaneous-arrival/right-hand-rule behavior has not yet been fully implemented and physically verified.
- Lane tracking is map- and localization-based; there is no camera lane-line correction.
- Individual QCar localization and steering differences have not been fully characterized.
- UDP communication is intentionally simple and has no message authentication, persistence, or redundant coordinator.
- YOLO and reservation decisions are not yet contextually fused.
- The coordinator is a single local point of failure.
- Physical testing has not yet produced a formal repeatable performance dataset.
- Path repair has been physically validated only on `.152`; `.151` has passed offline route validation but not the equivalent post-change physical test.

## Decision Record

| Decision | Reason |
|---|---|
| Use a shared JSON road-network file | Separates physical map facts and traffic rules from controller code and supports later zones |
| Preserve unavailable roads as disabled/inactive | Allows future physical-map changes without redesigning the model |
| Use fixed routes during early phases | Makes failures repeatable and avoids inaccessible road segments |
| Run the coordinator on the local computer | Provides automatic centralized arbitration and a single observable log |
| Start with first-arrival priority | Realistic enough for early stop-controlled testing and scalable beyond permanent car priority |
| Fail closed on stale ownership | Prevents another vehicle from being granted a possibly occupied zone |
| Continue clearing after communication loss inside CZ1 | Avoids stopping a vehicle in the conflict zone |
| Do not disable YOLO vehicle stopping | Preserves an independent collision safeguard |
| Tune deterministic control before adding learning | Establishes a measurable, interpretable baseline |
| Insert Phases 1A–1C before parking and multi-zone expansion | Aligns development with the supervisor's documentation, lane reliability, interaction, and quantitative-evaluation priorities |
| Validate generated paths before controller tuning | Prevents valid control logic from acting on malformed reference geometry |
| Repair only conservative non-anchor spike patterns | Avoids silently changing legitimate road geometry or route-node intent |

## Progress Log

### September 25, 2026 — Supervisor-Feedback Roadmap Revision

- Revised the roadmap so the project no longer advances directly from CZ1 to
  multiple conflict zones.
- Defined Phase 1A for source-code/architecture documentation and quantitative
  lane/trajectory reliability work.
- Defined Phase 1B for deliberately staged same-time CZ1 and lead/follow
  interactions.
- Defined Phase 1C for repeated trials and the required lane-error, waiting-time,
  crossing-success, recovery-time, and minimum-separation metrics.
- Deferred autonomous parking to Phase 2 and multiple conflict zones to Phase 3.
- Retained front CSI processing as measurement-only; camera steering and
  physical camera calibration are not assumed to be the next solution.
- Established the next-report priority order: documentation, baseline evidence,
  diagnosed lane improvement, controlled interaction, and quantitative comparison.

### September 21, 2026

- Created `EXPERIMENTAL_REPORTS.md` to preserve chronological procedures, evidence, failed approaches, quantitative comparisons, and planned experiments separately from the current-status overview.
- Performed the first single-vehicle lane test with `.152` on route `11 -> 12 -> 7 -> 14 -> 16 -> 18 -> 11`.
- Startup localization was plausible: position error `0.20 m` and heading error `0.6 degrees` at node 11.
- Tracking remained close to the path before the roundabout, generally showing cross-track errors below approximately `0.06 m`.
- Near the node-14-to-node-16 roundabout movement, the waypoint index jumped from `586` to `785` between status samples. Heading error immediately rose to approximately `174 degrees`, steering saturated at `+/-0.524 rad`, and cross-track error later exceeded `1 m`.
- Identified the cause as the new multi-segment waypoint-advancement loop repeatedly testing future curved segments against one unchanged vehicle position.
- Replaced it with bounded one-segment-per-control-update advancement and cyclic index normalization.
- Classified the run as a controller regression, not sufficient evidence of `.152` calibration failure.
- A second `.152` run showed that limiting advancement to one segment per 200 Hz update was insufficient. The waypoint index progressed from `587` to `770` much faster than the vehicle could physically travel, heading error exceeded `150 degrees`, and steering again saturated.
- Added an `0.08 m` waypoint-capture radius: advancement now requires the car to be both beyond the segment endpoint and physically near the next waypoint.
- Added a latched safety stop when absolute cross-track error exceeds `0.30 m` or absolute heading error exceeds `90 degrees`. A restart is required after this tracking fault.
- Added estimated position, next-waypoint distance, and tracking-fault state to the vehicle status log.
- The `.151` single-vehicle test should wait until these corrections are deployed and `.152` successfully passes the same roundabout section.
- A third `.152` run exposed a startup incompatibility in the `0.08 m` capture rule. The controller began with its front-axle tracking point approximately `0.389 m` beyond the first route waypoint, so `wp=0` never advanced and the car continued straight along the initial segment.
- Rolled back all experimental lane-control behavior to the last known SDCS baseline: Stanley gain `0.7`, projection-based one-segment advancement, original Stanley speed term, constant nominal route speed, and no latched tracking stop.
- Retained position, waypoint index/distance, cross-track error, and heading error as non-controlling diagnostics. Intersection reservation and safety changes were not rolled back.
- At that stage, the next experiment was defined as a `.152` baseline-characterization run, with further lane changes isolated and introduced one at a time.
- The restored baseline completed the roundabout physically but did not maintain its lane reliably.
- Quantitative review of that baseline status log appeared to show six impossible estimated-position jumps above `0.35 m`, including repeated `1.1–1.2 m` changes near the roundabout at approximately `0.25 m/s`; later raw-pose instrumentation did not reproduce those jumps.
- Recorded localization ambiguity as a hypothesis rather than a confirmed cause.
- Added measurement-only localization diagnostics and automatic raw LiDAR event capture. No control behavior was changed.
- Preserved a staged research sequence covering localization validation, calibration, robust measurement handling, waypoint management, controller comparison, speed planning, and eventual camera lane correction.
- A subsequent instrumented run produced no localization events: maximum post-startup position innovation was approximately `0.062 m`, and raw pose remained continuous.
- Located a deterministic generated-path defect at the node-16 join: adjacent turns of `-179.57 degrees` and `+179.57 degrees` caused a backward reference and route departure toward node 18.
- Added general path validation and conservative paired-reversal repair rather than a node-16-specific exception.
- Verified that the `.152` route preserves all route nodes and has a maximum adjacent turn of `1.03 degrees` after repair; the `.151` route remains unchanged.
- Added five path-manager tests and a raw-versus-prepared route plot. All 12 local automated tests pass.
- Physically tested the repaired `.152` route. The car continued from node 16 toward node 18, the former heading reversal did not recur, maximum logged cross-track error fell from `1.084 m` to `0.086 m`, and maximum logged heading error fell from `147.9 degrees` to `38.9 degrees`.
- Confirmed that the successful run contained no raw localization jump above `0.2 m` and no localization event capture; the route departure was caused by reference-path geometry rather than a localization event in that run.

### September 18, 2026

- Consolidated the current project status into this living document.
- Confirmed current vehicle assignments, routes, road availability, and launcher behavior from the repository.
- Phase 0 configuration and validation are present for CZ1.
- Phase 1 automatic coordination and QCar state machines are present.
- Six Phase 1 automated tests pass.
- Corrected permanent stale-owner lock recovery and lengthened communication tolerances.
- Added initial curve/lane-tracking experiments and quantitative status telemetry; the behavioral experiments were later rolled back after physical regressions, while the telemetry was retained.
- Identified lane accuracy and YOLO/reservation interaction as blocking work before multi-zone expansion; the revised roadmap now addresses these in Phases 1A–1C.

## How to Maintain This Document

Update this file whenever a meaningful change is made. Each update should include:

1. the date;
2. the problem or research question;
3. the code/configuration change;
4. the reason for the change;
5. how it was tested;
6. physical observations and quantitative results;
7. whether the issue is resolved, partially resolved, or still open; and
8. the resulting next action.

Do not describe an implemented feature as physically validated until it has been tested on the QCars. Preserve failed approaches and important defects in the challenge and progress sections because they provide useful engineering evidence for reports, presentations, and future research work.
