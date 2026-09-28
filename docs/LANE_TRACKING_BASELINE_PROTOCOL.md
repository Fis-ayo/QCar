# Lane-Tracking Baseline Protocol

## Purpose

This protocol establishes the unchanged waypoint-controller baseline required
before another lane-centering change is made. It is designed to answer two
different questions without confusing them:

1. **Path tracking:** How closely does the estimated QCar tracking point follow
   the reference supplied to the controller?
2. **Physical lane centering:** Is that reference, and the physical QCar body,
   actually centered inside the intended painted lane?

`cross_track_error_m` answers the first question only. It must not be reported
as physical lane-center error until the reference trajectory has been validated
against the physical lane center.

## Baseline freeze

Do not change the following between baseline trials:

- route node sequence;
- `tracking_offset_m`;
- Stanley/controller gains;
- nominal speed;
- path-repair behavior;
- localization/reference scan;
- tire/wheel configuration;
- QCar battery condition beyond normal charging between trials;
- physical start pose; or
- camera detector thresholds.

Record the Git commit identifier when one becomes available. Until then,
archive a copy of `config.txt`, `python/start.py`, `qcar/vehicle_control.py`,
`qcar/road_network.json`, and `qcar/path_manager.py` with each experiment set.

Current launcher baseline:

| Vehicle | Route | Tracking offset | Curvature controller | CSI diagnostics |
|---|---|---:|---|---|
| `.151` | `0 -> 2 -> 4 -> 6 -> 0` | `0.13 m` | Disabled | Enabled |
| `.152` | `11 -> 12 -> 7 -> 14 -> 16 -> 18 -> 11` | `0.20 m` | Disabled | Disabled |

These values document the present software; they are not yet accepted as the
best lane-centering parameters.

## Experimental order

Run one QCar at a time. Do not run the two-vehicle CZ1 scenario during baseline
collection.

1. Complete five valid `.151` trials.
2. Review the data for instrumentation or localization failures.
3. Configure `QCAR_IPS=[192.168.0.152]` and complete five valid `.152` trials.
4. Restore the intended configuration after collection.

A trial should contain at least one complete route lap. If it is safe and easy
to identify individual laps, collect three complete laps in a trial; otherwise
use one lap and repeat the trial five times. Never extend a run merely to meet
the target after unsafe behavior is observed.

## Pre-run checklist

- [ ] Test only one QCar; confirm the other vehicle cannot enter the track.
- [ ] Confirm the QCar IP, start node, route, and orientation.
- [ ] Confirm `config.txt` lists only the test vehicle.
- [ ] Verify the floor, mat, lighting, and movable objects have not changed.
- [ ] Check tires, visible steering alignment, battery condition, and sensor mounting.
- [ ] Start video with the complete tested route visible where practical.
- [ ] Record trial ID, date/time, vehicle, operator, and configuration snapshot.
- [ ] Keep emergency stopping immediately accessible.

## Route sections

Use the same section labels in observations, plots, and report tables.

### QCar `.151`

| Section | Movement | Classification | Known observation target |
|---|---|---|---|
| `151_S1` | node 0 to node 2 | Curved approach/lower loop | Early divider encroachment |
| `151_S2` | node 2 to node 4 | Tight lower-right curve | Crossing toward node-5 lane |
| `151_S3` | node 4 to node 6 | Side/upper transition | Recovery and centering |
| `151_S4` | node 6 to node 0 | CZ1 and return | Stop/restart and straight tracking |

### QCar `.152`

| Section | Movement | Classification | Known observation target |
|---|---|---|---|
| `152_S1` | node 11 to node 12 | Approach | Initial alignment |
| `152_S2` | node 12 to node 7 | CZ1/curved transition | Proximity to lane divider |
| `152_S3` | node 7 to node 14 | Curve/east approach | Tight entry and divider proximity |
| `152_S4` | node 14 to node 16 | Roundabout entry/circulation | Lane centering through curvature |
| `152_S5` | node 16 to node 18 | Roundabout exit | Correct branch and lane recovery |
| `152_S6` | node 18 to node 11 | Return | General centering and repeatability |

## Required evidence per trial

Preserve these files before the next run overwrites them:

- `vehicle_control.log`;
- `tracking_diagnostics.csv`;
- `localization_diagnostics.csv`;
- `localization_events/`, even if empty;
- `lane_diagnostics.csv`, `lane_camera.log`, and `lane_frames/` when enabled;
- synchronized video or a written explanation if video is unavailable;
- completed `trial_observations.csv`; and
- copies of the baseline configuration/source files listed above.

Suggested archive structure:

```text
experiments/
  BASELINE_2026-XX-XX/
    configuration/
    151_trial_01/
      vehicle_control.log
      tracking_diagnostics.csv
      localization_diagnostics.csv
      localization_events/
      lane_diagnostics.csv
      lane_camera.log
      lane_frames/
      trial_observations.csv
      video_notes.md
```

## Trial validity and safety stop rules

Stop the vehicle immediately if it:

- fully enters an unintended or opposing lane;
- leaves the drivable mat/road surface;
- approaches an obstacle without stopping;
- exhibits sustained maximum steering with increasing route error;
- reports a startup-pose mismatch or tracking/path safety error; or
- behaves differently enough that continued motion is not predictable.

A stopped unsafe run is still useful evidence, but mark it `SAFETY_STOP` rather
than a completed trial. Do not delete or replace it. A trial is invalid for the
performance average only when the experiment was not executed as configured,
instrumentation failed, or an unrelated external interruption occurred. State
the exclusion reason explicitly.

## Quantitative measurements

Calculate the following from `tracking_diagnostics.csv` for each complete trial
and each route section:

- mean absolute map-reference cross-track error;
- RMS map-reference cross-track error;
- 95th-percentile absolute map-reference cross-track error;
- maximum absolute map-reference cross-track error;
- mean and 95th-percentile absolute heading error;
- steering saturation percentage;
- maximum predicted steering demand;
- completion/safety-stop outcome; and
- lap duration when a full lap is completed.

Aggregate the five trials using the mean and standard deviation, while also
reporting the worst trial. Do not remove outliers without a documented sensor,
logging, or procedural failure.

The current `tools/analysis/analyze_tracking.py` reports several of these whole-run
metrics and produces a route/time plot. Per-section and multi-trial aggregation
will be added only after the baseline files establish the exact segmentation
and lap-boundary behavior.

## Physical lane-centering evidence

The current logs measure error relative to the controller reference, not the
painted lane center. During this baseline, record for every section:

- `CENTERED`, `LEFT_OF_CENTER`, `RIGHT_OF_CENTER`, `TOUCHING_DIVIDER`,
  `CROSSED_DIVIDER`, `TOUCHING_OUTER_BOUNDARY`, or `LEFT_ROAD`;
- approximate start/end time from the video or log;
- whether the front, center, rear, or entire QCar crossed the marking; and
- confidence of the observation (`HIGH`, `MEDIUM`, or `LOW`).

For `.151`, CSI yellow measurements may support visibility and image-space
alignment analysis, but `yellow_offset_normalized` is not a metric lateral
distance and must not be presented as meters.

## Reference-trajectory validation gate

Before tuning gains or enabling camera steering:

1. Plot the controller reference and estimated QCar trajectory.
2. Identify the logged waypoint/time corresponding to every observed lane
   boundary event.
3. Compare the configured reference with the intended physical lane center at
   those locations using known map geometry, measured checkpoints, or another
   defensible ground-reference method.
4. Classify each failure as primarily:
   - `REFERENCE_GEOMETRY`;
   - `LOCALIZATION`;
   - `CONTROLLER_TRACKING`;
   - `ACTUATOR/VEHICLE`;
   - `VEHICLE_FOOTPRINT`; or
   - `UNRESOLVED`.

If the reference itself is not validated, physical lane-center error remains
unavailable and only map-reference tracking metrics may be reported.

## Baseline completion criteria

The baseline stage is complete when:

- five valid or safely terminated trials exist for each QCar;
- every trial has the required logs and observation record;
- route sections and observed crossing times can be aligned to diagnostics;
- localization anomalies have been identified rather than silently treated as
  controller error;
- path-reference metrics are summarized without calling them physical
  lane-center error; and
- the evidence supports a specific next change or an explicit unresolved
  hypothesis.

No controller parameter or camera-control change should be introduced until
this baseline package has been reviewed.
