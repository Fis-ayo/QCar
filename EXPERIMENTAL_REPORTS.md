# Multi-QCar Experimental Reports

**Created:** September 21, 2026  
**Purpose:** Chronological record of physical and software experiments supporting the Multi-QCar autonomous-driving project  
**Current project overview:** See `PROJECT_STATUS.md`

## Document Scope

This file records experimental questions, configurations, observations, measurements, failures, conclusions, and follow-up actions. It is intended to preserve evidence that can later support a thesis, paper, presentation, or formal technical report.

`PROJECT_STATUS.md` remains the source of truth for the current system state and immediate next actions. This file preserves how conclusions were reached, including unsuccessful approaches.

The earliest experiments were discussed and observed before this document was created. Where exact timestamps, complete logs, or code commit identifiers were not retained, the report explicitly states that limitation rather than inventing missing information.

## Experimental Conventions

### Status labels

- **Completed:** Procedure and evidence are sufficient to answer the experiment's immediate question.
- **Partially completed:** Useful evidence was collected, but additional runs or artifacts are required.
- **Superseded:** The experiment remains historically relevant, but its software configuration is no longer active.
- **Planned:** Procedure has been defined but not performed.

### Evidence levels

- **Physical observation:** Behavior directly observed on the SDCS map.
- **Runtime log:** Evidence recorded by a QCar or the local coordinator.
- **Structured diagnostic:** CSV or binary sensor/state data recorded by instrumentation.
- **Offline validation:** Reproducible software test or geometric analysis performed without moving a QCar.

### Current vehicle identities

| Vehicle | IP address | Assigned start | Current route |
|---|---|---:|---|
| `.152` | `192.168.0.152` | Node 11 | `11 -> 12 -> 7 -> 14 -> 16 -> 18 -> 11` |
| `.151` | `192.168.0.151` | Node 0 | `0 -> 2 -> 4 -> 6 -> 0` |

The local coordinator runs at `192.168.0.153` on UDP port `18750`.

## Experiment Index

| ID | Experiment | Status | Principal result |
|---|---|---|---|
| EXP-001 | Deterministic two-car route operation | Completed | Both QCars moved simultaneously on fixed physical-map-compatible routes |
| EXP-002 | CZ1 first-arrival reservation | Partially completed | Both cars stopped; one received exclusive permission while the other waited |
| EXP-003 | Stale-owner coordinator recovery | Completed in software; physical endurance pending | Permanent stale lock was identified and corrected without granting a second owner |
| EXP-004 | Multi-waypoint advancement experiment | Superseded | Controller skipped far ahead around the roundabout and left the route |
| EXP-005 | One-waypoint-per-update experiment | Superseded | At 200 Hz, projection-only advancement still progressed unrealistically fast |
| EXP-006 | Waypoint capture-radius experiment | Superseded | Start reference remained at waypoint zero because the tracking point began beyond the capture area |
| EXP-007 | Restored baseline and localization hypothesis | Completed as diagnosis | Baseline completed the roundabout, but apparent pose jumps motivated raw localization instrumentation |
| EXP-008 | Raw localization/EKF instrumentation | Completed for `.152` comparison runs | Later runs showed continuous raw pose and small innovations; no localization event was captured |
| EXP-009 | Generated-route geometry analysis | Completed | A paired near-180-degree reversal was proven immediately before node 16 |
| EXP-010 | General route-integrity repair | Completed on `.152` | Repair preserved node 16 and restored progress toward node 18 |
| EXP-011 | `.151` independent post-repair baseline | Planned | Validate second route and compare vehicle-specific tracking behavior |
| EXP-012 | Two-car post-repair CZ1 endurance test | Planned | Reassess lane interaction, YOLO stops, fairness, and reservation release |
| EXP-013 | General lane-tracking diagnosis | Completed baseline comparison | Both cars follow correct references but exhibit a shared inward curve bias; `.152` additionally saturates steering in the roundabout |
| EXP-014 | Curvature-demand characterization | Completed on both vehicles | Missing curvature anticipation explains moderate-curve lag; accumulated feedback explains `.152` roundabout saturation |
| EXP-015 | Curvature-aware controller A/B test | Superseded after `.151` physical test | First candidate slightly worsened inward curve error; launcher returned to baseline |
| EXP-016 | Scheduled-feedback curvature controller | Superseded after `.151` physical test | Candidate 2 increased inward error by adding turn-in steering |
| EXP-017 | Curvature-scheduled feedback without feedforward | Software implemented; `.151` validation pending | Preserve straight tracking and let cross-track correction oppose inside curve cutting |
| EXP-018 | Vehicle-footprint and reference-point diagnosis | Baseline `.151` run completed | Forward control point remained near the route while the estimated body and inside-rear wheel cut much farther inward |
| EXP-019 | Tracking-offset A/B test | Completed; partial improvement | `0.13 m` removed the gradual early encroachment and reduced modeled curve errors, but `.151` still entered the node-5 lane on the tighter curve |
| EXP-020 | Front-CSI lane measurement validation | Implemented; stationary/physical run pending | Validate camera view, lane colors, ROI, two-boundary detection, center offset, heading, and confidence before steering fusion |
| EXP-021 | `.151` repeated unchanged-controller baseline | Completed; four clean runs plus one localization-disturbed run | Physical node-5 lane crossing repeated while map-reference tracking error remained comparatively small |
| EXP-022 | Bounded footprint-aware reference correction | Completed and rejected after two physical candidates | Both offsets reduced modeled inner error but failed physically and increased outer-boundary risk |
| EXP-023 | Yellow-boundary warning replay and Candidate 3 | Offline and saved-frame validation complete; live display confirmation pending | Candidate 3 warned `2.86 s` before shutdown while saved frames showed the real yellow divider crossing image center |
| EXP-024 | Candidate 4 camera speed safety | Implemented and validated offline; physical `.151` test pending | Existing runs predict slowdown `2.13-4.67 s` and latched stop `0.49-3.73 s` before operator shutdown |
| EXP-025 | Candidate 5 earlier camera safety thresholds | Implemented and validated offline; physical `.151` test pending | Moves slowdown to `-0.60`, crawl gain to `0.35`, and stop to `-0.40` after Candidate 4 stopped after crossing |

## EXP-001 — Deterministic Two-Car Route Operation

**Status:** Completed  
**Date:** Before September 18, 2026; exact run date was not retained  
**Evidence:** Physical observation and QCar runtime logs shared during development

### Research question

Can both QCars move simultaneously using repeatable routes that avoid unavailable physical road segments?

### Configuration

- `.152` assigned to start node 11.
- `.151` assigned to start node 0.
- Random route selection was disabled.
- Fixed routes were loaded from `qcar/road_network.json`.
- Unavailable nodes and edges were excluded.

### Observations

- Both vehicles moved under autonomous control.
- Both attempted to follow their generated waypoint routes.
- Repeated operation eventually brought the vehicles into a shared area where they collided.

### Conclusion

Deterministic route execution solved the initial inability to reproduce behavior, but simultaneous waypoint following alone did not provide intersection safety.

### Resulting action

Define shared conflict zones and introduce explicit intersection ownership.

## EXP-002 — CZ1 First-Arrival Reservation

**Status:** Partially completed  
**Date:** Before September 18, 2026; exact run date was not retained  
**Evidence:** Physical observation, `vehicle_control.log`, and `intersection_coordinator.log`

### Research question

Can two autonomous QCars stop at the central intersection and receive mutually exclusive first-arrival permission?

### Configuration

- Conflict zone: `CZ1_CENTRAL`.
- Control type: three-way all-way stop with western approach retained but inactive.
- Policy: first arrival, with fairness support for another car already approaching.
- Coordinator: local UDP service.
- Vehicle states: `CRUISING`, `APPROACHING`, `WAITING`, `GRANTED`, `OCCUPYING`, and `EXITED`.

### Observations

- Both cars reached CZ1 and stopped.
- One vehicle received the reservation and moved first.
- The second vehicle remained stopped while CZ1 was owned.
- In an early extended run, the waiting vehicle did not proceed soon enough before the first vehicle returned on another lap, contributing to another conflict.

### Conclusion

Exclusive reservation was demonstrated, but release timing, fairness, communication recovery, route accuracy, and long-duration interaction still required improvement.

### Resulting action

Add explicit exit polygons, next-car preference, detailed coordinator logs, and stale-owner recovery.

## EXP-003 — Stale-Owner Coordinator Recovery

**Status:** Completed in software; physical endurance validation remains  
**Date:** September 18, 2026  
**Evidence:** Coordinator log and six Phase 1 automated tests

### Research question

Why did both vehicles remain stationary after `.152` arrived first and received CZ1 ownership?

### Evidence

The coordinator log showed:

```text
CZ1_CENTRAL: granted to 192.168.0.152
CZ1_CENTRAL: owner 192.168.0.152 is stale; zone locked for safety
CZ1_CENTRAL: queued 192.168.0.151
```

The vehicle continued running, but the coordinator safety lock could previously be cleared only by an `EXITED` message.

### Root cause

When the current owner resumed sending fresh heartbeats while still waiting, ownership remained assigned but `blocked_by_stale_owner` remained true. The owner could not use its grant, and no other car could receive one.

### Change

- Clear the stale-owner safety lock only when the same current owner resumes fresh communication.
- Keep every other car blocked during the interruption.
- Increase coordinator stale tolerance from `1.0` to `3.0` seconds.
- Increase QCar grant validity from `0.5` to `1.5` seconds.

### Validation

- Added an automated stale-lock recovery test.
- All six Phase 1 tests pass.
- A later coordinator log showed communication restoration clearing the lock without granting a second owner.

### Conclusion

The deadlock was a state-recovery defect rather than an intersection-geometry failure. The correction preserves fail-closed behavior.

## EXP-004 — Multi-Waypoint Advancement Experiment

**Status:** Superseded and rolled back  
**Date:** September 21, 2026  
**Evidence:** `.152` `vehicle_control.log`

### Research question

Would allowing the controller to skip every already-passed waypoint prevent the reference from lagging after localization corrections?

### Change under test

The waypoint controller repeatedly advanced within one control update while the vehicle pose projected beyond the current segment.

### Observations

Near the roundabout:

```text
wp=586, crossTrack=0.003 m, headingError=8.1 deg
wp=785, crossTrack=-0.200 m, headingError=174.1 deg
```

- The reference jumped almost 200 waypoints.
- Steering saturated at `+/-0.524 rad`.
- Cross-track error later exceeded `1 m`.
- The vehicle left the intended track.

### Root cause

One unchanged vehicle position can project beyond the endpoint planes of many tangent segments around a curve. Repeated evaluation in one update incorrectly advanced the reference around much of the roundabout.

### Conclusion

Unbounded multi-waypoint advancement is unsafe for curved and self-near paths. The change was rolled back.

## EXP-005 — One-Waypoint-Per-Update Experiment

**Status:** Superseded and rolled back  
**Date:** September 21, 2026  
**Evidence:** `.152` `vehicle_control.log`

### Research question

Would limiting advancement to one waypoint per control update prevent the roundabout reference jump?

### Observations

```text
wp=587, crossTrack=-0.037 m, headingError=6.0 deg
wp=617, crossTrack=-1.046 m, headingError=31.9 deg
wp=719, crossTrack=-0.279 m, headingError=115.6 deg
wp=770, crossTrack=-0.054 m, headingError=153.4 deg
```

The controller runs at 200 Hz. Even one advancement per update allowed many indices to be crossed within each half-second log interval while the projection condition remained true.

### Conclusion

Rate limiting by control iteration alone does not establish valid route progress. A progress manager must use path geometry and bounded forward context.

## EXP-006 — Waypoint Capture-Radius Experiment

**Status:** Superseded and rolled back  
**Date:** September 21, 2026  
**Evidence:** `.152` `vehicle_control.log`

### Research question

Would requiring proximity to the next waypoint prevent projection-based advancement across curved paths?

### Change under test

Waypoint advancement required the car to be beyond the segment endpoint and within `0.08 m` of the next waypoint.

### Observations

```text
wp=0, wpDistance=0.389 m
wp=0, wpDistance=0.753 m
wp=0, wpDistance=1.374 m
wp=0, wpDistance=1.756 m
```

The steering controller uses a tracking point `0.2 m` ahead of the estimated vehicle center. At startup this point was already beyond the initial waypoint region, so the proximity condition could never be recovered. The vehicle continued following the direction of the first segment.

### Conclusion

A capture radius cannot be added without a correct initial route-progress estimate. The experiment was rolled back along with the associated curve-speed, gain, low-speed-softening, and tracking-fault changes.

## EXP-007 — Restored Baseline and Localization Hypothesis

**Status:** Completed as a diagnostic stage  
**Date:** September 21, 2026  
**Evidence:** Baseline `.152` vehicle log

### Research question

What does the original SDCS Stanley controller do when all experimental lateral-control changes are removed?

### Configuration

- Stanley gain: `0.7`.
- Nominal speed: `0.25 m/s`.
- Original projection-based waypoint advancement.
- Original Stanley speed term.
- Added logging only; no lateral behavior modification.

### Observations

- `.152` traversed the roundabout physically but did not remain centered in its lane.
- One status log appeared to contain six estimated-position changes above `0.35 m`, including changes of approximately `1.1–1.2 m`.
- Cross-track and heading errors became large after the apparent changes.

### Interpretation at the time

LiDAR scan-match ambiguity or another pose discontinuity was recorded as a hypothesis. It was not accepted as a root cause without raw measurement evidence.

### Resulting action

Add measurement-only raw scan-matched pose, EKF, motion, and LiDAR diagnostics.

## EXP-008 — Raw Localization and EKF Instrumentation

**Status:** Completed for the comparison runs; instrumentation remains active  
**Date:** September 21, 2026  
**Evidence:** `localization_diagnostics.csv`, `vehicle_control.log`, and empty `localization_events` directories

### Research question

Do large route errors begin with raw scan-matched pose jumps, EKF corrections, or waypoint/path behavior?

### Instrumentation

For every new scan-matched pose, record:

- raw position and heading;
- EKF state before correction;
- EKF state after correction;
- position and heading innovation;
- speed, steering, and gyroscope yaw rate; and
- LiDAR valid-point and range summaries.

Capture recent full LiDAR scans if position innovation reaches `0.35 m` or heading innovation reaches `45 degrees`.

### Results from the pre-repair instrumented run

- 244 structured pose records.
- Median position innovation: approximately `0.020 m`.
- Maximum post-startup position innovation: approximately `0.062 m`.
- No raw-position jump above `0.2 m`.
- No localization event capture.

### Results from the post-repair successful run

- 325 structured pose records.
- Median position innovation: approximately `0.020 m`.
- Maximum post-startup position innovation: approximately `0.085 m`.
- Maximum post-startup heading innovation: approximately `1.9 degrees`.
- Median valid LiDAR points: `303`.
- No raw-position jump above `0.2 m`.
- No localization event capture.

### Conclusion

The instrumented comparison runs did not reproduce a large raw localization discontinuity. Localization remains a monitored subsystem, but the node-16 route departure in these runs was not initiated by a captured localization event.

## EXP-009 — Generated-Route Geometry Analysis

**Status:** Completed  
**Date:** September 21, 2026  
**Evidence:** Raw generated waypoint coordinates and offline geometric analysis

### Research question

Why did `.152` leave the intended route after completing the roundabout and reaching node 16?

### Evidence

The raw route contained:

```text
803: (0.906995, 3.713979)
804: (0.908554, 3.704101)
805: (0.907610, 3.709805)  # node 16
806: (0.909262, 3.699331)
```

Segment directions were:

```text
803 -> 804: -81.03 degrees
804 -> 805: +99.40 degrees
805 -> 806: -81.03 degrees
```

This produced adjacent heading changes of approximately `-179.57 degrees` and `+179.57 degrees`.

### Runtime correlation

Immediately before failure:

```text
wp=801, crossTrack=0.023 m, headingError=35.6 degrees
wp=804, crossTrack=-0.043 m, headingError=-147.9 degrees
```

Raw pose remained smooth and GPS innovation was small. Waypoint 804 then remained selected while cross-track error grew beyond `1 m`.

### Root cause

The generated path briefly moved past node 16, reversed to the exact node coordinate, and reversed again toward node 18. The baseline controller acted on this backward-pointing reference segment.

### Conclusion

The node-16 departure was a reference-path integrity failure, not evidence that Stanley gain alone was incorrect.

## EXP-010 — General Route-Integrity Repair

**Status:** Completed and physically validated on `.152`  
**Date:** September 21, 2026  
**Evidence:** Five path-manager tests, route comparison plot, `.152` runtime log, and localization CSV

### Research question

Can unsafe generated-route spikes be detected and repaired generally without adding a node-16-specific rule or changing lateral control?

### Software change

Added `qcar/path_manager.py` between `SDCSRoadMap.generate_path()` and the Stanley controller.

The path manager:

- validates finite 2D coordinates;
- rejects zero-length segments and excessive gaps;
- detects near-180-degree heading reversals;
- verifies route closure and route-node representation;
- identifies consecutive opposite reversals;
- protects exact route-node anchors;
- removes only a non-anchor spike candidate; and
- prevents startup if the prepared route remains invalid.

No change was made to:

- Stanley gain (`0.7`);
- nominal speed (`0.25 m/s`);
- EKF correction;
- LiDAR scan matching;
- intersection behavior; or
- baseline waypoint progression.

### Offline results

| Measurement | Raw `.152` route | Prepared `.152` route |
|---|---:|---:|
| Waypoint count | 992 | 991 |
| Maximum adjacent heading change | `179.57 degrees` | `1.03 degrees` |
| Maximum waypoint gap | `0.0182 m` | `0.0182 m` |
| Route nodes preserved | All | All |

The non-anchor point at original index 804 was removed. Node 16 remained. The `.151` route required no repair and remained unchanged offline.

### Physical results

| Measurement | Before repair | After repair |
|---|---:|---:|
| Maximum logged cross-track error | `1.084 m` | `0.086 m` |
| Maximum logged heading error | `147.9 degrees` | `38.9 degrees` |
| Stuck at the node-16 join | Yes | No |
| Continued from node 16 toward node 18 | Failed | Successful |
| Raw localization jump above `0.2 m` | None in instrumented comparison | None |
| Localization event capture | None | None |

The successful run continued beyond the repaired area:

```text
wp=789, crossTrack=-0.021 m, headingError=38.1 degrees
wp=805, crossTrack=-0.005 m, headingError=35.9 degrees
wp=818, crossTrack=-0.025 m, headingError=12.2 degrees
wp=831, crossTrack=-0.029 m, headingError=-4.6 degrees
wp=880, crossTrack=0.005 m, headingError=-10.2 degrees
wp=894, crossTrack=-0.004 m, headingError=-6.7 degrees
```

### Conclusion

The physical result supports the general principle that perception, planning, and reference outputs must be validated before tuning control. The controller had been responding to malformed input geometry. General lane centering remains open because steering still reached its limit through tight parts of the roundabout.

### Limitations

- Physical validation has been performed on `.152`, not yet `.151`.
- Only the configured routes have been evaluated.
- Automatic repair intentionally covers only conservative paired-reversal spikes.
- The successful result is not yet a repeatability dataset.
- Lane-centering error and steering saturation remain to be characterized independently.

## EXP-011 — `.151` Independent Post-Repair Baseline

**Status:** Planned

### Research question

Does `.151` complete its offline-valid route consistently, and how does its lane-centering behavior compare with `.152`?

### Planned procedure

1. Configure only `.151` in `config.txt`.
2. Place it at node 0 with the expected orientation.
3. Run at least one complete lap without changing Stanley gain or nominal speed.
4. Preserve `vehicle_control.log` and `localization_diagnostics.csv` before another run.
5. Record visible inward/outward lane offset and any steering saturation.
6. Compare cross-track, heading, localization innovation, and physical lane position with `.152`.

### Acceptance and investigation criteria

- Route validation reports zero repairs for `.151`.
- No unexplained waypoint lock or large index progression occurs.
- Any consistent one-sided physical drift is investigated as possible steering or vehicle calibration difference.
- Controller changes are not introduced until the baseline comparison is complete.

## EXP-012 — Two-Car Post-Repair CZ1 Endurance Test

**Status:** Planned

### Research question

After individual route behavior is characterized, can both cars repeatedly use CZ1 without collision, starvation, stale-lock deadlock, or avoidable YOLO blockage?

### Planned measurements

- arrival time at the stop line;
- grant and release time;
- waiting duration;
- conflict-zone occupancy duration;
- stale-owner or communication events;
- lane position of the yielding vehicle;
- YOLO vehicle-stop events;
- number of completed laps; and
- collision, near-collision, and emergency-stop events.

## EXP-013 — General Lane-Tracking Diagnosis

**Status:** Baseline comparison completed on both vehicles  
**Date:** September 23, 2026  
**Evidence:** `tracking_diagnostics.csv` will be produced by each physical run

### Research question

Are the observed lane departures caused by the generated reference path,
localization, route progress, lateral control/steering response, or a
vehicle-specific physical difference?

### Current physical observations

- `.152` follows its intended route but is not consistently centered through
  curves.
- `.151` follows route `0 -> 2 -> 4 -> 6 -> 0`, but has been observed moving
  toward the lane containing node 5 around the node-2-to-node-4 curve rather
  than passing node 4 as intended.
- These are observations, not yet root-cause conclusions.

### General instrumentation

Added a route-independent recorder that logs at 20 Hz:

- EKF vehicle-center pose and the controller's forward tracking point;
- active waypoint and segment endpoints;
- projected reference point and reference heading;
- cross-track and heading errors;
- distance to the next waypoint;
- actual and requested speed, throttle, and steering;
- steering saturation; and
- intersection phase.

The recorder does not modify control. The same code runs on both cars and on
every route. `tools/analysis/analyze_tracking.py` produces common metrics and a
trajectory/time-series plot from the resulting CSV.

### Planned procedure

1. Run `.151` alone from node 0 without changing controller parameters.
2. Preserve its tracking, localization, and vehicle logs and note the physical
   time/location of the lane departure. Stop the car if continuing is unsafe.
3. Repeat once to test whether the departure is deterministic.
4. Run `.152` alone under the same software and logging conditions.
5. Compare both cars using the same metrics and plots.

### Interpretation criteria

- A reference trajectory entering the unintended lane indicates a map or
  path-generation issue.
- A correct reference accompanied by a displaced pose indicates localization
  or calibration error.
- A correct, stable pose and reference with persistent tracking error or
  steering saturation indicates controller, speed, steering calibration, or
  mechanical response.
- An implausible waypoint change indicates route-progress failure.
- Similar behavior on both cars supports a shared software/model cause;
  materially different behavior supports per-vehicle calibration or hardware
  as a contributing factor. Any eventual correction should remain general and
  parameterized rather than tied to a particular node.

### First `.151` result

The car was stopped when it physically crossed toward the lane containing node
5. The captured run lasted from approximately `1.00 s` to `14.54 s` and
progressed monotonically from waypoint 1 to waypoint 403.

- The node-4-side controller reference remained on the intended outer path,
  reaching approximately `x = 2.255 m`.
- The controller tracking point remained approximately `0.048–0.080 m` inside
  that reference through the measured eastern curve.
- Maximum east-curve cross-track magnitude was `0.080 m`; the median was
  approximately `0.059 m`.
- No steering saturation occurred in the eastern-curve samples.
- Waypoint progress did not reverse or jump to a distant segment.
- Localization recorded no outlier event. The last correction before the stop
  had `0.015 m` position innovation and approximately `0.3 degrees` heading
  innovation with 292 valid LiDAR points.
- The warning at segment 574–575 was not active during the observed departure;
  the run ended at waypoint 403.
- Repeated YOLO status-check errors did not command a stop in this interval;
  the recorded speed gain remained `1.0`.

This result argues against a node-4-specific wrong-path selection. It supports
a persistent inward tracking/alignment error, but does not yet distinguish a
shared lateral-controller limitation from map-to-physical alignment,
scan-matching bias, or a vehicle-specific steering/mechanical difference. A
repeat `.151` run and comparable `.152` data are still required.

### Second `.151` result

The repeat run reproduced the physical lane crossing in the same lower-right
curve and was stopped immediately. The run ended at waypoint 381.

- The outer reference again remained correct at approximately `x = 2.255 m`.
- Eastern-curve median cross-track magnitude was approximately `0.055 m` and
  the maximum was `0.090 m`, comparable to the first run's persistent inward
  offset.
- Waypoint progress was monotonic and did not select a node-5 path.
- No steering saturation occurred in the eastern curve.
- No localization event was triggered. A `0.039 m` localization innovation
  near `12.31 s` temporarily increased the measured tracking error, but the
  inward offset was already present before that correction.

The repeated outcome establishes that `.151`'s inward curve behavior is
reproducible. The next required evidence is the equivalent `.152` independent
run; controller changes remain deferred until that comparison is complete.

### `.152` comparison result

The independent `.152` run completed one route cycle and began another. Its
validated route included the previously established general paired-reversal
repair near node 16. Localization remained stable: after startup, median
position innovation was approximately `0.018 m`, the 95th percentile was
`0.041 m`, maximum was `0.062 m`, maximum heading innovation was approximately
`1.95 degrees`, and no localization event was triggered.

Moving samples from the first route cycle showed:

| Route region | Median absolute cross-track | Maximum | Negative-error samples | Steering saturation |
|---|---:|---:|---:|---:|
| Node 11 toward node 7 / central curve | `0.039 m` | `0.080 m` | `94.2%` | `0%` |
| Node 7 toward node 14 / eastern approach | `0.042 m` | `0.087 m` | `98.8%` | `0%` |
| Roundabout | `0.030 m` | `0.070 m` | `92.5%` | `25%` |
| Node 18 toward node 11 | `0.016 m` | `0.046 m` | `78.9%` | `0%` |

Roundabout heading error reached approximately `39 degrees`. The route index
decrease in the full log corresponds to the normal cyclic wrap after one lap,
not backward route progress.

### Baseline conclusion

Both physical vehicles follow their intended software reference but favor the
same inward side on curves. This makes isolated node selection an unlikely
primary cause and supports a shared curved-path tracking limitation. The
roundabout additionally exceeds the available steering authority at the
current speed/reference behavior. The next design step should therefore be a
general curvature-aware investigation—measuring reference curvature and
testing curvature feedforward and speed scheduling under controlled A/B
conditions—rather than adding corrections for nodes 4, 5, 7, or 14.

## EXP-014 — Curvature-Demand Characterization

**Status:** Curvature characterization completed on both vehicles  
**Date:** September 23, 2026  
**Evidence:** Extended `tracking_diagnostics.csv`

### Research question

Do upcoming path curvature and predicted steering demand explain the shared
inward curve error and `.152` roundabout saturation?

### Implementation

The route-independent tracking recorder now calculates:

- signed local path curvature;
- strongest signed curvature within the next `0.50 m`;
- local and upcoming bicycle-model steering demand using the documented QCar
  wheelbase of `0.256 m`; and
- whether predicted upcoming steering exceeds the existing `30 degree`
  steering-command limit.

Curvature is measured across a spatial window rather than adjacent waypoint
triples so that dense waypoints and very short segments do not dominate the
estimate. These fields are measurement-only: steering, speed, Stanley gain,
route progress, and localization remain unchanged.

### Planned procedure

1. Run `.151` independently until the previously observed lower-right curve
   behavior is captured.
2. Run `.152` independently through the ordinary curves and roundabout.
3. Compare upcoming curvature and predicted steering demand with the onset of
   cross-track error, heading error, and actual steering saturation.
4. Use the evidence to define and A/B test bounded curvature feedforward and
   curve-speed scheduling as general route-independent changes.

### Preliminary `.152` result

The first curvature-instrumented `.152` run completed one route cycle and
began another. The principal measurements from moving samples were:

| Route region | Median absolute cross-track | Maximum | Maximum predicted steering | Actual steering saturation |
|---|---:|---:|---:|---:|
| Node 11 toward node 7 | `0.030 m` | `0.080 m` | `16.0 degrees` | `0%` |
| Node 7 toward node 14 | `0.040 m` | `0.082 m` | `20.7 degrees` | `0%` |
| Roundabout | `0.032 m` | `0.071 m` | `22.4 degrees` | `24.5%` |
| Node 18 toward node 11 | `0.012 m` | `0.057 m` | `18.5 degrees` | `0%` |

No predicted curvature demand exceeded the existing `30 degree` steering
limit, even though the Stanley output saturated repeatedly in the roundabout.
This indicates that geometric curvature demand alone does not exceed the
vehicle command range. Instead, accumulated heading and cross-track feedback
drives the existing controller into saturation. Consequently, curvature
feedforward must be combined with bounded/rebalanced feedback rather than
simply added to the present saturated command. The `.151` comparison remains
required before selecting parameters or enabling control changes.

### `.151` comparison result

The curvature-instrumented `.151` run reproduced the inward departure on the
lower-right node-2-to-node-4 curve and was stopped at waypoint 385.

- Median absolute curve error was approximately `0.049 m`, the 95th percentile
  was `0.076 m`, and the maximum was `0.082 m`.
- Approximately `98%` of curve samples had the same inward sign.
- Maximum predicted geometric steering demand was approximately `16.0
  degrees`; no predicted demand exceeded the `30 degree` limit and no actual
  steering saturation occurred in the curve.
- Upcoming demand was already approximately `16 degrees` while the local path
  segment was still nearly straight. The baseline controller has no curvature
  term and therefore waited for heading/cross-track error before responding.
- Across the analyzed transition, actual steering averaged approximately
  `10.4 degrees` while upcoming geometric demand averaged approximately `16.0
  degrees`.
- Localization remained stable: median post-startup position innovation was
  approximately `0.022 m`, its 95th percentile was `0.048 m`, its maximum was
  `0.070 m`, maximum heading innovation was approximately `1.37 degrees`, and
  no event was captured.

### Curvature-characterization conclusion

The two-car evidence supports a shared mechanism. The baseline controller
reacts to cross-track and heading error but does not anticipate known route
curvature. Moderate curves therefore develop a repeatable inward offset before
sufficient steering is commanded. In the tighter `.152` roundabout, the
accumulated feedback later saturates even though geometric steering demand
remains below the nominal limit. The appropriate next experiment is a
conservative, bounded controller combining smoothly applied curvature
feedforward, Stanley feedback, and curvature-based speed scheduling, with an
explicit switch allowing baseline-versus-modified A/B runs.

## EXP-015 — Curvature-Aware Controller A/B Test

**Status:** Superseded after first `.151` physical A/B test  
**Date:** September 23, 2026  
**Evidence:** Automated tests and future physical tracking logs

### Implementation under test

The optional `curvature_aware` mode:

- previews path curvature approximately `0.20 m` ahead for steering;
- applies `75%` of predicted geometric steering as a low-pass-filtered,
  rate-limited feedforward term;
- retains Stanley cross-track feedback with a bounded low-speed denominator;
- reduces heading-feedback gain to avoid adding full feedforward on top of the
  previously saturated response;
- uses the maximum curvature in the next `0.50 m` to reduce nominal speed
  progressively from `0.25 m/s` to no less than `0.18 m/s`; and
- preserves the existing `30 degree` steering limit and intersection/YOLO
  stop authority.

The launcher exposes a per-vehicle `curvature_control` setting. `True` enables
the candidate controller and `False` restores baseline steering and nominal
speed behavior. Tracking logs now identify the controller mode and separately
record feedforward, feedback, and curve-speed target values.

### Initial physical procedure

1. Run `.151` alone in curvature-aware mode and stop if its lower-right-curve
   behavior is unsafe.
2. Compare its curve error and lane position with the three preserved baseline
   runs.
3. If the result is safe and improved, run `.152` alone through the full route
   and roundabout.
4. Compare ordinary-curve error, roundabout heading error, saturation rate,
   and minimum commanded speed with the baseline.

### `.151` physical result

The candidate mode was active and the car again crossed toward the node-5
lane. Relative to the comparable baseline curve run:

| Measurement | Baseline | Candidate |
|---|---:|---:|
| Median absolute curve error | `0.049 m` | `0.058 m` |
| 95th-percentile curve error | `0.076 m` | `0.083 m` |
| Maximum curve error | `0.082 m` | `0.091 m` |
| Curve steering saturation | `0%` | `0%` |

Candidate curve speed averaged approximately `0.20 m/s`. Feedforward averaged
approximately `8.5 degrees`, but reduced feedback averaged only `2.6 degrees`,
giving approximately `11.1 degrees` total steering. The baseline had averaged
approximately `10.4 degrees`, so the first candidate produced very little net
increase despite the added preview term. Localization remained stable and no
outlier event was captured.

### Conclusion

The first curvature-aware candidate did not fix lane crossing and slightly
worsened numerical tracking. Reducing heading feedback globally canceled most
of the intended feedforward benefit. The candidate remains in source code for
controlled research, but both launcher entries have been returned to baseline
(`curvature_control: False`) so it cannot be used accidentally. A second
candidate must change the way feedforward and feedback are blended rather than
merely increasing a single gain.

## EXP-016 — Scheduled-Feedback Curvature Controller

**Status:** Superseded; numerical error improved but physical lane crossing remained  
**Date:** September 23, 2026  
**Evidence:** Seventeen automated tests and future physical logs

### Change from candidate 1

Candidate 1 reduced heading feedback everywhere, which canceled most of its
feedforward benefit. Candidate 2 instead:

- raises curvature feedforward gain from `0.75` to `0.85`;
- keeps heading gain relatively strong on moderate curves;
- schedules heading gain progressively downward only as preview curvature
  approaches the tight-roundabout range;
- retains filtered/rate-limited feedforward, the `30 degree` output bound, and
  `0.18–0.25 m/s` curvature-based speed scheduling; and
- logs the active heading gain for verification.

The launcher enables candidate 2 only for `.151`. `.152` remains on baseline
until `.151` demonstrates safe improvement. The log must report
`curvature_aware_v2` during the candidate test.

### Acceptance criteria

- The car must not cross into the node-5 lane.
- Median and maximum node-2-to-node-4 curve errors should improve over the
  baseline values of approximately `0.049 m` and `0.082 m`.
- Steering must remain bounded without unstable oscillation or an outward-lane
  departure.

### Physical result

Candidate 2 failed acceptance. The car crossed toward node 5, and error began
growing in the earlier bottom curve as physically observed. Node-2-to-node-4
median error increased to approximately `0.076 m` and maximum to `0.103 m`.
The earlier bottom-curve median was approximately `0.087 m`, with a `0.138 m`
maximum. Localization remained stable and the candidate mode was active.

The signed geometry clarified the incorrect assumption: on these leftward
curves, additional positive feedforward turns the car farther toward the
inside divider. The heading term already favors the inside turn, while the
negative cross-track term is attempting to move the car outward. Candidate 2
was therefore superseded without testing on `.152`.

## EXP-017 — Curvature-Scheduled Feedback Without Feedforward

**Status:** Software implemented; `.151` physical validation pending  
**Date:** September 23, 2026

Candidate 3 removes curvature steering feedforward. It preserves full heading
gain on straight segments, where physical centering is already good, and uses
preview curvature only to reduce heading gain progressively on curves. This
allows the existing cross-track term to oppose inward corner cutting. The
proven curvature-based speed reduction and all steering/safety bounds remain.

The mode is logged as `curvature_feedback_v3`, is enabled only for `.151`, and
must be stopped if the car crosses either the center divider or outside road
edge.

### First `.151` result

Candidate 3 produced the first clear numerical improvement:

| Region | Baseline median | Candidate 3 median | Baseline maximum | Candidate 3 maximum |
|---|---:|---:|---:|---:|
| Earlier bottom curve | `0.070 m` | `0.034 m` | `0.119 m` | `0.125 m` |
| Node 2 toward node 4 | `0.049 m` | `0.020 m` | `0.082 m` | `0.069 m` |

The candidate-3 early-region maximum occurred immediately after startup near
waypoint 41; the largest later early-curve error was approximately `0.086 m`.
Node-2-to-node-4 95th-percentile error improved from approximately `0.076 m`
to `0.037 m`. Localization remained stable, no outlier event was captured,
and waypoint progress remained monotonic.

The subsequent physical observation confirmed that `.151` still crossed into
the node-5 lane. It showed better boundary behavior on the node-0-to-node-2
curve, but did not satisfy the primary lane-containment criterion. Candidate 3
is therefore superseded and disabled in the launcher despite its numerical
improvement.

This disagreement between a roughly `0.020 m` logged tracking-point error and
physical body encroachment indicates that steering gain alone is no longer the
right variable to tune. The controller tracks a point `0.20 m` ahead of the
EKF pose, while the platform configuration documents a `0.256 m` wheelbase and
the EKF bicycle model itself uses `0.20 m`. The next diagnostic must compare
the reference with the estimated vehicle center/front/rear axle and swept
footprint, and verify map-to-physical lane alignment.

## EXP-018 — Vehicle-Footprint and Reference-Point Diagnosis

**Status:** Baseline `.151` run completed and analyzed  
**Date:** September 23, 2026

### Research question

Why does the physical vehicle cross a lane boundary when the controller's
forward tracking point reports a small path error?

### Implementation

The tracking CSV now estimates body-center, front/rear axle, and all four
wheel-center positions and their signed lateral errors. It also records the
inside of the current curve and the most inward wheel error.

The geometry uses Quanser's documented body frame: axle centers at `+/-0.130
m` and half wheel track of `0.086 m`. The output explicitly labels
`ekf_pose_assumed_body_center`, because verifying the EKF/GPS pose origin is
part of the experiment. Both vehicles are returned to baseline control; this
instrumentation does not alter driving behavior.

### `.151` procedure

Run `.151` alone on baseline and stop immediately after physical lane crossing.
Compare tracking-point, body-center, axle, and inner-wheel errors around the
observed event. Use the result to decide whether to correct the control point,
model a curvature-dependent body-envelope offset, or investigate physical-map
alignment before adding camera lane correction.

### Results

The baseline run progressed through waypoint 349 on route `0 -> 2 -> 4 -> 6
-> 0`. For moving samples in the node-2-to-node-4 curve (waypoints 200-400):

- forward control-point absolute error: median `0.050 m`, 95th percentile
  `0.070 m`, maximum `0.084 m`;
- assumed body-center absolute error: median `0.120 m`, 95th percentile
  `0.139 m`, maximum `0.147 m`;
- rear-axle-center absolute error: median `0.166 m`, 95th percentile `0.189
  m`, maximum `0.196 m`;
- inside-wheel absolute error: median `0.248 m`, 95th percentile `0.271 m`,
  maximum `0.276 m`.

The curve was left-hand, making the rear-left wheel the estimated most inward
point. Near the end of the run, the forward control-point error improved from
approximately `-0.055 m` to `-0.014 m`, while the assumed body-center error
remained approximately `-0.136 m` to `-0.083 m`. Localization was stable:
post-startup GPS position innovation had a median of approximately `0.020 m`,
a 95th percentile of `0.052 m`, and a maximum of `0.067 m`; no localization
events were generated.

### Conclusion

The logged controller error does not represent the QCar's full swept
footprint on curves. The forward point can follow the route reasonably closely
while the body and rear inside wheel cut substantially farther inward. This
matches the observed pattern of good straight-line centering and curve-only
lane encroachment and is stronger evidence than another steering-gain change.

The conclusion remains conditional on the explicitly logged assumption that
the EKF pose is the body center. The next controlled step is to verify that
pose origin, then parameterize the hard-coded `0.20 m` forward offset and A/B
test the documented front-axle offset (`0.13 m`) without changing gains,
speed, route, or waypoint logic.

## EXP-019 — Tracking-Offset A/B Test

**Status:** Completed; partial improvement, lane-containment criterion failed  
**Date:** September 23, 2026

### Research question

Does moving the Stanley tracking point from `0.20 m` to the documented `0.13
m` body-center-to-front-axle distance reduce inward body off-tracking on
curves?

### Configuration

- Vehicle: `.151` only
- Route: `0 -> 2 -> 4 -> 6 -> 0`
- Candidate tracking offset: `0.13 m`
- Baseline tracking offset: `0.20 m`
- Curvature controller: disabled
- Speed, Stanley gain, path, and waypoint logic: unchanged

The launcher retains `.152` at `0.20 m`. `vehicle_control.py` now accepts a
bounded `--tracking_offset_m` argument and reports the active value at startup.

### Safety and acceptance criteria

Stop `.151` if it fully enters the node-5 lane or otherwise leaves the safe
drivable surface. Preserve the vehicle, tracking, and localization logs and
record the physical observation. Accept the candidate only if visible lane
containment and estimated body-envelope errors improve without unstable
steering or a new outward-boundary departure.

### Results

The startup log confirmed baseline steering with a `0.130 m` tracking offset.
The physical car ran closer to the outer edge, and the gradual inward crossing
previously observed on node 0 to node 2 no longer occurred. It still crossed
into the node-5 lane on the tighter node-2-to-node-4 curve.

For moving samples at waypoints 200-400, compared with the prior `0.20 m`
baseline footprint run:

- tracking/front-axle median absolute error improved from about `0.050 m` to
  `0.033 m`;
- assumed body-center median absolute error improved from about `0.120 m` to
  `0.068 m`;
- rear-axle median absolute error improved from about `0.166 m` to `0.102 m`;
- inside-wheel median absolute error improved from about `0.248 m` to `0.189
  m`;
- steering did not saturate in this waypoint range.

Localization remained stable: position innovation median was approximately
`0.023 m`, its 95th percentile was `0.050 m`, and its maximum was `0.073 m`.
No localization events were captured.

### Conclusion

The front-axle offset is a useful general geometry correction, but it is not a
complete lane-centering solution. It improved both physical behavior and the
modeled footprint while failing the decisive physical lane-containment test.
Map-relative errors alone do not measure the QCar's position between the
painted physical lane boundaries. The next experiment should add camera lane
measurement in observation-only mode, synchronize it with tracking telemetry,
and determine whether physical lane-center error disagrees systematically
with the map reference before camera feedback is allowed to steer.

## EXP-020 — Front-CSI Lane Measurement Validation

**Status:** Yellow-line detector revision implemented; second `.151` measurement run pending  
**Date:** September 23, 2026

### Research question

Can the QCar2 front CSI camera provide a sufficiently reliable direct
measurement of physical lane-center and lane-heading error on this map?

### Implementation

`lane_camera_server.py` opens only the front CSI camera through
`QCarCameras(enableFront=True)`. `lane_detection.py` applies an initial
yellow/white HSV segmentation inside a configurable road ROI, searches for
left and right boundary candidates, and calculates center offset, normalized
offset, heading error, confidence, and colored-pixel counts. It records
`lane_diagnostics.csv`, periodic annotated images in `lane_frames/`, and
runtime messages in `lane_camera.log`.

This subsystem is measurement-only and has no connection to steering or
throttle. The launcher enables it only on `.151` for the first validation;
`.152` remains unchanged. YOLO continues using the separate RealSense
RGB/depth interface. The local observer now has a `Front Lane Measurement`
display when `.151` is selected as the probing car.

### First validation procedure

1. Place `.151` centered on a representative straight lane and run it as the
   only selected QCar, preferably as the probing car.
2. Confirm that `lane_camera.log` contains `[LANE]` records and that the
   observer or saved frames show the intended front view and road ROI.
3. Initially keep the car stationary long enough to capture several images.
4. If the view is correct, perform one supervised route run and stop at the
   usual safety boundary.
5. Preserve `lane_camera.log`, `lane_diagnostics.csv`, representative images
   from `lane_frames/`, and the existing vehicle/tracking/localization logs.

No lane-center value will be used for control until these images establish the
correct camera orientation, ROI, markings, and detection convention.

### First physical results

Only the saved images timestamped around 3:27 PM were used for this analysis.
The front CSI camera produced the correct forward view and the configured ROI
covered the nearby road. Yellow-line segmentation was visually useful.
However, the broad white HSV mask repeatedly classified the bright raised
curb/platform on the right as a white lane boundary. Several
`detected=True` results were therefore false two-boundary detections.

Across 2,212 camera records, the reported detection rate was `85.5%`, but this
must not be treated as lane accuracy. During the synchronized start/node-0-to-
node-2 portion, the reported rate was `93.8%` and the median white-pixel count
was about `127,331`, reflecting the large curb region. In the node-2-to-node-4
region, reported detection fell to `44.5%`. The present center-offset and
heading values are consequently not valid control signals.

Localization remained stable during the run: median position innovation was
approximately `0.020 m`, its 95th percentile was `0.051 m`, its maximum was
`0.063 m`, and no localization events were captured.

### Revised direction

The next detector must save both raw and annotated frames and stop treating
all low-saturation bright pixels as lane boundaries. The most defensible next
baseline is the QCar2 example's yellow-line geometry (curve/slope and lateral
position) with temporal continuity and a confidence gate. White-boundary
detection should require thin, line-like geometry after camera
undistortion/perspective handling rather than a filled color region. LaneNet
can then be evaluated offline on the same raw frames as a competing detector.
Map tracking remains the fallback at intersections, branches, or frames where
the visual measurement is ambiguous.

### Implemented revision

The measurement subsystem now identifies the best thin, vertically continuous
yellow component, fits a quadratic image-space curve, and logs its near/far
positions, normalized lateral position, heading, curvature, row coverage, fit
RMSE, temporal jump, and confidence. Large filled yellow regions are rejected,
and sudden frame-to-frame jumps reduce confidence. The legacy white/yellow
two-boundary result remains in the CSV only for comparison.

Each periodic snapshot now produces a clean image in `lane_frames/raw/` and a
separate overlay in `lane_frames/annotated/`. The offline tool
`tools/analysis/analyze_lane_frames.py` can rerun detector revisions over saved raw
frames without another physical drive. A sanity check on the 33 requested
3:27 PM annotated snapshots accepted 17 yellow-line observations with median
accepted confidence of approximately `0.87`; this is not an accuracy result
because those older images already contain overlays. The next raw-frame run is
the validation evidence.

### Second `.151` measurement run

The supplied folder contained two image runs (approximately 3:53 PM and 3:58
PM). Analysis used the newer 3:58 PM images and matching CSV/vehicle logs.
During the interval overlapping vehicle tracking, yellow detection was present
in `55.4%` of start/node-0-to-node-2 samples and `86.9%` of node-2-to-node-4
samples. Median accepted confidence was approximately `0.86` and `0.83`,
respectively, with median fit RMSE near `5 px`.

Visual inspection confirms that the blue fit generally follows the yellow
marking. It also reveals an important limitation: on tight curves the fixed
near-field evaluation point can extrapolate the fitted curve beyond the image
boundary (including normalized offsets below `-1.0`) while confidence remains
high. After waypoint 400, no yellow observation was accepted during the
remaining synchronized samples because the marking was no longer visible
through enough of the configured ROI. Confidence currently measures fit
quality and continuity, not whether the evaluated yellow position is a valid,
usable lane-control reference.

Localization was stable: position innovation median was approximately `0.020
m`, its 95th percentile `0.054 m`, its maximum `0.072 m`, and no events were
captured. Camera and vehicle timestamps aligned within approximately `0.038
s` during their overlapping interval.

The next detector revision must reject off-image extrapolation, evaluate only
within the actually observed component span, and compensate for the strong
front-camera distortion/perspective before interpreting image slope as lane
heading. This remains measurement-only.

### Observed-span offline correction

The yellow-line evaluator now uses the 20th and 90th percentiles of the rows
actually occupied by the selected component instead of fixed image rows. Both
evaluation points must lie inside the image; off-image estimates are rejected.
The CSV explicitly records observed near/far rows, extrapolation status, and
`perspective_calibrated=0`.

Offline reprocessing of only the 34 newer 3:58 PM raw snapshots accepted 30
(`88.2%`). Accepted normalized yellow positions stayed within the image
(`-0.991` to `0.099`) rather than producing the previous below-`-1.0`
extrapolations. The resulting analysis is preserved under
`Logs/QCar .151/offline_1558_observed_span/`.

Inspection of the installed `QCarCameras` implementation found no exposed CSI
intrinsic matrix or distortion coefficients, and no project calibration file
was available. Thus `yellow_heading_rad` remains an image-space feature angle,
not a metric road-heading error. A physical front-camera calibration and
ground-plane mapping are required before this measurement can be converted
into a steering correction.

## EXP-021 — `.151` Repeated Unchanged-Controller Baseline

**Status:** Completed  
**Date:** September 25, 2026

### Research question

Does `.151` cross toward the node-5 lane because of inconsistent tracking and
localization, or can the physical departure repeat while the QCar remains close
to the reference supplied to its controller?

### Frozen configuration and procedure

- Vehicle: `192.168.0.151`
- Route: `0 -> 2 -> 4 -> 6 -> 0`
- Controller: baseline Stanley-style controller
- Tracking offset: `0.13 m`
- Curvature controller: disabled
- Front CSI detector: measurement-only
- Five separate runs; the operator stopped every run when the QCar crossed
  into the node-5 lane

No vehicle-control parameter was intentionally changed between trials. The
trials are classified as valid safety-terminated diagnostic runs rather than
completed laps.

### Evidence completeness

All five folders contain vehicle, tracking, localization, lane-camera, lane
CSV, and raw/annotated frame evidence. Tracking plots were generated in each
folder. The `localization_events` folders were empty for all trials. Trial 5's
localization CSV nevertheless records one event-capture flag; the expected NPZ
event artifact was not available.

### Whole-run map-reference results

| Trial | Duration (s) | Final waypoint | Mean absolute CTE (m) | RMS CTE (m) | 95th percentile (m) | Maximum (m) | Mean heading error (deg) | Steering saturation | Classification |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---|
| 1 | 14.75 | 420 | 0.044 | 0.052 | 0.107 | 0.135 | 16.1 | 3.3% | Clean |
| 2 | 13.17 | 375 | 0.041 | 0.051 | 0.122 | 0.133 | 16.7 | 3.7% | Clean |
| 3 | 13.28 | 378 | 0.042 | 0.052 | 0.122 | 0.132 | 17.2 | 2.8% | Clean |
| 4 | 14.52 | 413 | 0.046 | 0.053 | 0.121 | 0.133 | 16.1 | 3.0% | Clean |
| 5 | 14.14 | 403 | 0.049 | 0.058 | 0.125 | 0.174 | 16.4 | 4.3% | Localization disturbed |

Across clean Trials 1–4, mean absolute CTE was
`0.0435 +/- 0.0021 m` and RMS CTE was `0.0521 +/- 0.0011 m` (mean plus/minus
sample standard deviation). Steering saturation was `3.18 +/- 0.41%`.

### Vulnerable curve and localization evidence

For the historically vulnerable node-2-to-node-4 range (waypoints 200–400),
clean-trial mean absolute map-reference CTE was only `0.0269–0.0367 m`, with
maximum values of `0.0689–0.0814 m`. These are smaller than the preceding
waypoint-0-to-199 mean errors of `0.0543–0.0574 m` even though every physical
run crossed into the node-5 lane.

Clean Trials 1–4 recorded no localization events. Their maximum post-startup
position innovations were `0.0643–0.1117 m`. Trial 5 recorded alternating raw
Y estimates near elapsed times `11.42` and `11.62 s`, producing position
innovations of approximately `0.356` and `0.364 m`. The operator did not notice
an unusual physical motion in that trial, and its physical outcome matched the
other four runs. It is retained as evidence but excluded from the clean-trial
aggregate.

### Conclusion

The node-5 lane crossing is repeatable and is not adequately explained by
random controller tracking error or by the Trial 5 localization disturbance.
The controller generally remains close to the path it has been given in the
same region where the physical vehicle crosses the lane boundary. Therefore,
the next testable hypothesis is that the generated/offset reference, its
registration to the physical mat, or the modeled vehicle footprint is not
placing the physical QCar at the intended lane center through the curve.

The reported CTE values are map-reference errors, not physical lane-center
errors. Another Stanley gain or camera-steering change is not justified until
the physical placement of the reference through node 2 to node 4 is validated.

### Map-derived vehicle-footprint overlay

`tools/visualization/plot_reference_validation.py` now overlays all five estimated body
trajectories, the prepared route, the median tracking point, and median front
and rear wheel trajectories on the SDCS road graph. The resulting plot is
stored at `Logs/QCar .151/reference_validation_151_node2_to_node4.png`.

The SDCS roadmap constants give an outer-to-inner lane-center separation of
approximately `0.2696 m` in this curve. If the painted divider is at the
midpoint, it is approximately `0.1348 m` inward from the route centerline. With
the modeled QCar half-track of `0.086 m`, the body center has only about
`0.0488 m` of inward margin before an inside wheel reaches that midpoint.

For clean Trials 1–4 at waypoints 200–400, mean absolute modeled body-center
error was `0.0611–0.0700 m`, already larger than that approximate margin. The
modeled inside-wheel trajectory lay beyond the map-derived midpoint throughout
the analyzed samples. This agrees with the operator's observation that the
physical QCar entered the node-5 lane and provides a geometric explanation for
why a relatively small tracking-point CTE can coexist with a lane violation.

The midpoint layer is explicitly map-derived and unmeasured. It is not yet
accepted as painted-line ground truth. A physical lane-width/divider check is
still required before using it to set a control offset or acceptance threshold.

### Physical lane-width validation

Physical measurements supplied after the overlay analysis were:

- node-4 to node-5 lane-center spacing: approximately `11 in` (`0.2794 m`);
- each lane center to the divider center: approximately `5.5 in` (`0.1397 m`);
- painted divider width: approximately `0.5 in` (`0.0127 m`); and
- the lower-right curve uses the same center-to-divider spacing.

The near edge of the painted divider is therefore approximately `5.25 in`
(`0.13335 m`) from the intended lane center. This agrees within approximately
`1.5 mm` with the map-derived route-to-divider estimate of `0.1348 m` and
validates the overlay geometry to the precision of the supplied measurements.

Using the modeled `0.086 m` half-track, only approximately `0.04735 m` remains
between an ideally oriented inside wheel center and the near divider edge. Tire
width and a desired safety clearance would reduce the usable margin further.
The clean-trial body-center deviations of `0.0611–0.0700 m` exceed this nominal
margin, consistent with the repeated physical lane crossing.

This measurement strengthens the footprint/off-tracking explanation. It does
not by itself select a controller implementation, but it supports testing a
route-independent curvature/footprint-aware reference correction rather than
a node-specific waypoint edit.

### Next action

Implement a bounded, route-independent footprint-aware curve correction using
the physically validated divider clearance. Keep the baseline controller and
speed otherwise unchanged, log the applied correction, and repeat the same
`.151` safety-supervised trials before enabling camera steering.

## EXP-022 — Bounded Footprint-Aware Reference Correction

**Status:** First physical candidate failed lane containment but improved modeled footprint position  
**Date:** September 25, 2026

### Hypothesis

The forward tracking point follows the route while the vehicle body and rear
footprint cut inward on curves. A small curvature-derived outward displacement
of the local control reference should reduce this off-tracking without changing
straight-road behavior or introducing node-specific waypoint edits.

### Implementation

- Added `qcar/footprint_correction.py`.
- For signed curvature `k`, the target outward displacement is based on
  `sqrt(R^2 + L^2) - R`, where `R = 1/|k|` and wheelbase `L = 0.256 m`.
- Correction is zero below `0.08 1/m`, symmetric for left/right turns, smoothed
  with a `0.20 s` time constant, rate-limited to `0.12 m/s`, and capped at
  `0.04 m`.
- The correction shifts the local projected reference outward; waypoint
  progression and reference heading remain unchanged.
- Nominal speed, Stanley gain, tracking-point offset, camera authority, path
  repair, YOLO behavior, and intersection behavior remain unchanged.
- Added target/applied correction fields to `tracking_diagnostics.csv` and the
  periodic vehicle status log.
- Added `--footprint_correction` as an opt-in vehicle-control flag. It is enabled
  only for `.151` in the launcher for the first supervised comparison.
- Added four unit tests covering straight behavior, left/right symmetry,
  maximum offset, smoothing, and rate limiting. The complete suite passes all
  27 tests.

### First-run safety procedure

Run `.151` alone from node 0. Stop immediately for divider crossing, outer-edge
contact, route departure, sustained saturated steering, or unpredictable
behavior. Preserve all logs and state whether the node-2-to-node-4 curve was
completed within the intended lane. Do not run five trials until the first
candidate run has been reviewed.

### First physical result

The startup log confirmed `baseline+footprint_reference_v1`, and the requested
and applied correction both reached approximately `-0.036 m` through the
node-2-to-node-4 left curve. Localization was stable: maximum post-startup
position innovation was `0.0851 m` and no event was captured. The operator
stopped the run because the QCar still failed to remain inside the intended
lane.

Compared with the mean of clean EXP-021 Trials 1–4 over waypoints 200–400:

| Measurement | EXP-021 baseline | EXP-022 candidate 1 | Change |
|---|---:|---:|---:|
| Tracking-point mean absolute CTE | `0.0324 m` | `0.0339 m` | Essentially unchanged |
| Body-center mean absolute error | `0.0666 m` | `0.0407 m` | Improved by `0.0259 m` |
| Front-left wheel mean absolute error | `0.1148 m` | `0.0911 m` | Improved by `0.0237 m` |
| Rear-left wheel mean absolute error | `0.1837 m` | `0.1547 m` | Improved by `0.0290 m` |
| Front-right wheel mean absolute error | `0.0504 m` | `0.0745 m` | Shifted outward by `0.0241 m` |

The correction direction is supported: the inside footprint moved outward by
roughly `2.4–2.9 cm` without materially degrading tracking-point error. The
magnitude was insufficient. The modeled inside rear wheel remained about
`0.1547 m` from the route center, approximately `0.0214 m` beyond the measured
near divider edge at `0.13335 m`, before including tire width or safety margin.

Candidate 1 must not be accepted as a lane-keeping solution. A larger bounded
candidate may be justified only after confirming from physical observation
that the outward edge retained adequate clearance and the crossing was reduced
rather than worsened.

### Operator comparison and decision

The operator observed that Candidate 1 crossed into the node-5 lane later than
the baseline, confirming that its outward direction had a beneficial effect.
At the beginning of motion the QCar was close to the outer road boundary, but
the operator subsequently clarified that this startup behavior also existed
before Candidate 1. The vehicle moved fully inside its lane before eventually
crossing toward node 5.

`tools/analysis/analyze_footprint_envelope.py` was added to test both sides against the
measured `0.13335 m` center-to-near-boundary distance. In waypoints 200–400,
the modeled four-wheel envelope was narrow enough to fit in the lane for 100%
of samples in all four clean baselines and Candidate 1. Baseline negative-side
violations occurred in `94.3–100%` of samples with no positive-side violations.
Candidate 1 reduced negative-side violations to `79.0%`; positive-side
violations occurred in `1.7%` of samples. Thus the primary repeated failure
remains the node-5 side, and the geometry does not show that the QCar is too
wide to fit when laterally centered.

Candidate 2 increases the curvature correction gain from `1.0` to `1.65` and
the cap from `0.04 m` to `0.06 m`. Both values are explicit launcher arguments,
and the controller mode is `baseline+footprint_reference_v2`. All other control
and safety settings remain unchanged. This is an incremental supervised test,
not an accepted fleet-wide setting.

### Candidate 2 physical result

Candidate 2 reached the intended approximately `0.059 m` correction with stable
localization (`0.0714 m` maximum post-startup innovation and no event capture).
The QCar still crossed into the node-5 lane at the lower-right curve, and the
operator reported that the pre-existing outer-edge behavior became noticeably
worse. Candidate 2 was therefore rejected and disabled in the launcher.

The measured-boundary envelope comparison changed as follows:

| Result over waypoints 200–400 | Candidate 1 | Candidate 2 |
|---|---:|---:|
| Negative/node-5-side violations | `79.0%` | `50.9%` |
| Positive/outer-side violations | `1.7%` | `16.4%` |
| Mean translation still needed to center envelope | `0.0401 m` | `0.0148 m` |
| Complete envelope geometrically able to fit | `100%` | `100%` |

Candidate 2 again moved the footprint in the intended direction but traded one
boundary failure for another without preventing the physical node-5 crossing.
This rejects further increases to a fixed curvature-only offset.

### Camera/tracking alignment check

`tools/analysis/analyze_camera_tracking_alignment.py` synchronized Candidate 2 CSI and
tracking samples by wall time. Yellow detection was present for all 110
selected samples. The yellow image-space offset had moderate correlation with
negative clearance (`0.529`) and positive clearance (`-0.646`), whereas the
legacy two-boundary center offset had weak negative-clearance correlation
(`0.099`). Median yellow offset also differed between inside-lane, negative-
violation, and positive-violation groups.

This is evidence that the camera contains useful local boundary information,
but it is not sufficient to authorize steering. The yellow component can
change identity or geometry, image offsets are not metric distances, and the
result is from one candidate run. The next camera step should validate stable
boundary identity and directional warning thresholds offline across baseline
and candidate data before any bounded steering fusion.

## EXP-023 — Measurement-Only Yellow-Boundary Warning

**Status:** Offline replay and Candidate 3 saved-frame validation complete; live display confirmation pending  
**Date:** September 25, 2026  
**Evidence:** Seven `.151` lane/tracking datasets, `qcar/lane_warning.py`,
`tools/replay/replay_lane_warning.py`, and automated tests

### Research question

Can the existing front-CSI yellow-boundary measurement provide a repeatable
warning before the operator-observed node-5 lane crossing, without changing
vehicle control?

### Method and result

Each recorded run ended when the operator stopped `.151` after observing the
lane crossing. This stop time is therefore an approximate event marker, not a
frame-accurate boundary annotation. A warning is entered after six consecutive
frames with yellow confidence at least `0.60` and normalized yellow offset at
least `-0.32`. Once entered, it is latched for the rest of the diagnostic run
because the boundary may leave the camera view after crossing.

Offline replay produced a warning in all seven available datasets. The first
warning occurred `1.41-4.21 s` before the recorded stop, and the latched warning
was active throughout the final second in every run. The measurement-only
pipeline now records the warning and its debounce state in
`lane_diagnostics.csv` and overlays it on the probing display.

### Conclusion and limitation

The camera signal is sufficiently repeatable for a supervised real-time
warning experiment. It is not yet validated as metric lane clearance, and it
does not authorize camera steering or automatic braking. The next physical
test should verify that the warning occurs while the yellow boundary identity
is correct and before visible lane entry. Video or an observer timestamp is
needed to replace the approximate operator-stop label.

### Candidate 3 physical run

Candidate 3 retained baseline waypoint steering and did not enable the rejected
footprint correction, camera steering, or camera braking. The warning first
latched at camera elapsed time `30.20 s`, approximately `2.86 s` before the
vehicle-control log ended and near waypoint `304` in the node-2-to-node-4
curve. At entry, yellow confidence was `0.84` and normalized yellow offset was
`-0.249`.

Saved annotated frames show that the fitted feature was the physical yellow
divider. Its offset progressed from `-0.436` before warning to `-0.121`,
`-0.013`, and then `+0.323`, showing the divider moving across the image
center. Post-startup localization remained stable: maximum position innovation
was approximately `0.059 m`, with no localization event. Over waypoints
200-400, map-reference absolute cross-track error averaged approximately
`0.027 m` and reached `0.074 m` despite the camera-observed boundary approach.

The live observer display was unavailable, so the operator could not confirm
seeing the red warning during motion. Candidate 3 therefore validates boundary
identity and log timing from synchronized saved evidence, but not
human-observed real-time warning behavior.

### Leading solution direction

The strongest supported direction is staged hybrid navigation. Waypoints and
localization retain responsibility for global route and intersection movement;
the front CSI camera supplies local physical-boundary evidence. The proposed
authority is incremental: measurement/warning first, then a bounded safety
slowdown or stop, and only after repeated validation a small bounded steering
correction. Camera measurements must not select a different route or override
YOLO and intersection safety stops.

## EXP-024 — Candidate 4 Camera Slowdown and Latched Stop

**Status:** Implemented and validated offline; physical `.151` test pending

**Date:** September 28, 2026

**Evidence:** `qcar/lane_safety.py`, `tools/replay/replay_lane_safety.py`, eight
recorded `.151` datasets, and 39 automated tests

### Research question

Can the validated yellow-boundary signal prevent continued unsafe travel by
reducing speed and stopping, without granting the camera steering authority?

### Implementation

- `NORMAL`: speed gain `1.0`.
- `SLOWDOWN`: normalized yellow offset at least `-0.45`, confidence at least
  `0.60`, for four consecutive frames; speed gain `0.45`.
- `STOP`: offset at least `-0.15` for three consecutive reliable frames;
  speed gain `0.0` and latched until process restart.
- Slowdown release requires eight reliable frames beyond the `-0.55` clear
  margin. Weak/missing detections do not immediately release slowdown.
- Stale local camera communication for more than `0.50 s` fails closed with a
  zero speed gain when Candidate 4 is enabled.
- Camera state is transported locally over UDP port `18760`, combined with
  YOLO and intersection gains using the minimum restriction, and logged in
  both lane and tracking CSV files.
- Steering remains entirely waypoint-based.

### Offline replay result

Across Candidate 1/2/3 and five baseline datasets, slowdown first occurred
`2.13-4.67 s` before the recorded operator shutdown and STOP occurred
`0.49-3.73 s` before shutdown in every run. This establishes reproducible
software timing against existing evidence, but shutdown is only an approximate
crossing marker.

### Safety limitation and next test

Candidate 4 is enabled only for `.151` through an explicit launcher setting.
A critical stop cannot steer the stationary car back into its lane; the
operator must stop the experiment, reposition the vehicle, and restart. The
first physical test must determine whether slowdown and STOP occur before
physical divider entry and record false interventions. This is not yet an
accepted lane-centering solution.

### Candidate 4 physical result

Candidate 4 reduced speed as designed but stopped only after the vehicle had
crossed into the node-5 lane. Slowdown began near waypoint `287` at offset
`-0.427`; STOP reached vehicle control near waypoint `357` at offset `-0.150`.
The controller received the stop about `35 ms` after the camera decision, speed
fell below `0.02 m/s` within approximately `69 ms`, and estimated motion after
the stop command was only about `0.0024 m`. Post-startup localization innovation
remained below approximately `0.059 m`, with no event capture.

This rejects braking, transport delay, and localization discontinuity as the
main reason for the late stop. The `-0.15` image threshold did not represent
the physical vehicle boundary; camera perspective and vehicle footprint meant
the car crossed while the yellow divider was still left of image center.

## EXP-025 — Candidate 5 Earlier Camera Safety Thresholds

**Status:** Implemented and validated offline; physical `.151` test pending

**Date:** September 28, 2026

### Change

- Slowdown threshold changed from `-0.45` to `-0.60`.
- Slowdown speed gain changed from `0.45` to `0.35` (`0.0875 m/s` at the
  current nominal speed).
- Clear margin changed from `-0.55` to `-0.70`.
- Latched stop threshold changed from `-0.15` to `-0.40`.
- Confidence, debounce, stale-communication stop, logging, and waypoint-only
  steering remain unchanged.

### Offline validation

Replay across all nine available `.151` datasets placed slowdown around
waypoints `268-302` and STOP around waypoints `289-327`. On the physical
Candidate 4 data, the revised logic would have slowed near waypoint `268` and
stopped near waypoint `289`, rather than continuing to the observed Candidate
4 stop near waypoint `357`. All 39 automated tests pass.

### Next action

Run `.151` alone and record whether Candidate 5 stops fully inside its lane,
on the divider, or after crossing. An early but safe stop is acceptable for
this conservative test; autonomous recovery remains outside this stage.

### Candidate 5 physical result

Candidate 5 stopped earlier than Candidate 4, but the operator observed that
the left front and rear wheels were already beyond the yellow divider when it
stopped. Slowdown began near waypoint `268` at yellow offset `-0.572`. STOP was
generated at offset `-0.374` and reached vehicle control near waypoint `309`.
Vehicle speed fell from the reduced-speed condition to below `0.02 m/s` in
approximately `67 ms`; estimated displacement after the first logged STOP
sample was approximately `0.0007 m`. Post-startup localization innovation was
at most approximately `0.063 m`, with no event capture.

Candidate 5 is an improvement in stop location but fails the physical safety
criterion. Another uncalibrated threshold change is not justified. The next
step is the stationary measured-clearance protocol in
`docs/LANE_KEEPING_ACCEPTANCE_PLAN.md`, followed by a threshold or clearance
model derived from physical tire position.

## Current Experimental Conclusions

1. Fixed routes make multi-vehicle behavior reproducible.
2. Explicit CZ1 ownership is necessary; simultaneous route following alone is unsafe.
3. Reservation logic must recover safely from temporary communication loss.
4. Controller behavior cannot be diagnosed reliably without validating its reference path.
5. Unbounded or iteration-limited waypoint advancement is not a sufficient route-progress solution.
6. A proximity gate also requires correct initial route progress.
7. Raw localization instrumentation is necessary to distinguish pose failures from planning and control failures.
8. The node-16 route departure was caused by a deterministic paired reversal in generated waypoint geometry.
9. A general, conservative path-repair layer restored `.152` route progress without changing Stanley gain or speed.
10. General lane-centering accuracy remains the active research problem before the controlled Phase 1B two-vehicle demonstrations.
11. Five repeated `.151` safety-stop trials confirm that physical node-5 lane entry can occur while map-reference tracking remains small and repeatable; physical reference placement must therefore be validated before further gain tuning.
12. A debounced front-CSI yellow-boundary warning preceded all seven recorded `.151` operator stops in offline replay, but remains measurement-only pending real-time physical validation.

## Evidence Preservation Requirements

For every future physical experiment, preserve:

- `vehicle_control.log`;
- `localization_diagnostics.csv`;
- `localization_events/` if it contains event files;
- `intersection_coordinator.log` for multi-car tests;
- vehicle identity and start node;
- route sequence;
- relevant configuration files;
- date and approximate start time;
- physical observations;
- video when practical; and
- the code revision or commit identifier.

Store artifacts in a structure such as:

```text
experiments/
    EXP-013_2026-09-XX_short-description/
        README.md
        vehicle_152.log
        localization_152.csv
        localization_events_152/
        coordinator.log
        configuration/
        video-notes.md
```

Do not overwrite or combine files from separate runs. If no event files were produced, record that the event directory was empty rather than treating it as missing data.

## Template for Future Experiments

```markdown
## EXP-XXX — Title

**Status:** Planned | Partially completed | Completed | Superseded  
**Date:** YYYY-MM-DD  
**Evidence:** List logs, structured data, plots, and video

### Research question

What specific question does this experiment answer?

### Hypothesis

What result is expected, and why?

### Configuration

- Vehicle(s):
- Start node(s):
- Route(s):
- Controller parameters:
- Coordinator parameters:
- Calibration/reference files:
- Code revision:

### Controlled variables

What must remain unchanged?

### Procedure

1. Step one.
2. Step two.

### Safety conditions

- Manual stop condition:
- Automatic stop condition:
- Maximum run duration:

### Results

Record observations and quantitative measurements.

### Analysis

Explain what the evidence supports and what it does not establish.

### Conclusion

Answer the research question.

### Limitations

List missing evidence, confounding factors, and scope restrictions.

### Next action

State the single most useful follow-up experiment or implementation change.
```
