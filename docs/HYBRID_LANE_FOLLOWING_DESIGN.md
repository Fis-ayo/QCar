# Hybrid Lane-Following Design

## Objective

Improve physical lane centering without allowing camera perception to choose a
route or override intersection and obstacle safety. The map/localization
controller remains responsible for where the QCar travels. The front CSI
camera supplies bounded lateral guidance within that route.

This design intentionally avoids switching between two independent steering
controllers. It produces one final steering command with explicit ownership,
confidence, limits, and failure behavior.

## Control architecture

```text
road_network.json + GPS/LiDAR/EKF
        |
        v
validated route -> waypoint/Stanley steering -----------+
                                                         |
front CSI -> lane geometry -> confidence -> bounded -----+--> steering fusion
                                      camera correction  |          |
                                                         |          v
YOLO + CZ reservation + lane safety ---------------------+--> speed/safety arbiter
                                                                    |
                                                                    v
                                                               QCar actuators
```

### Authority boundaries

- Road-map and waypoint tracking select the route, direction, and intersection
  movement.
- Camera guidance may correct lateral placement only; it may not advance
  waypoints, select turns, or alter reservation state.
- YOLO and intersection restrictions may reduce speed to zero regardless of
  steering guidance.
- Camera confidence loss may reduce speed or remove camera authority. It must
  not create an abrupt steering step.
- The final steering command always retains a hard mechanical/software limit.

## Signals

### Map controller output

- waypoint index and route identifier;
- cross-track and heading errors;
- waypoint steering command;
- localization freshness and innovation diagnostics.

### Camera observation

- two-boundary detection state;
- confidence;
- normalized lane-center offset;
- image-space lane heading;
- left/right boundary locations;
- observation age;
- perspective-calibration state and calibration identifier.

### Hybrid decision

- raw and bounded camera candidate;
- slew-limited candidate;
- camera authority in `[0, 1]`;
- applied camera correction;
- final steering command;
- decision state and reason.

Each quantity is logged separately. This makes it possible to prove whether an
event was caused by the route reference, localization, camera measurement,
fusion policy, or actuator saturation.

## Steering law

The intended active form is:

```text
camera_candidate = sign * (K_lateral * lane_offset + K_heading * lane_heading)
camera_correction = authority * bounded_and_slew_limited(camera_candidate)
final_steering = clamp(waypoint_steering + camera_correction)
```

The gains and sign are not accepted merely because they exist in another QCar
project. They must be verified from stationary measurements and shadow data on
this physical installation.

## Operating modes

### Disabled

No camera-guidance process is required. Waypoint behavior is unchanged.

### Shadow

Camera observations and proposed corrections are transmitted and logged beside
the waypoint command. Camera authority is hard-coded to zero, so the proposed
correction cannot reach the actuator. This is the current implementation.

### Limited authority

This future mode may be enabled only after calibration and shadow acceptance.
Authority ramps smoothly from zero and is capped. The map controller remains
primary.

### Unavailable/degraded

Stale, incomplete, low-confidence, or geometrically invalid observations
remove camera authority. Any speed response is handled by the safety arbiter,
not hidden inside the steering calculation.

## Calibration requirements

Before nonzero authority, collect stationary data on both QCars at:

- centered straight lane positions;
- known left/right offsets from lane center;
- the node-0-to-node-2 curve;
- the node-2-to-node-4 lower-right curve;
- the node-11-to-node-7 area;
- the node-7-to-node-14 approach; and
- representative roundabout positions.

For every pose, record the vehicle-centerline-to-divider distance, camera
observation, vehicle heading, road section, camera identity, and image. Record
tire-to-divider clearance separately when validating safety. The resulting
calibration must define the expected camera center/heading measurements and
their relationship to physical clearance. A calibration identifier must be
carried in every guidance packet.

## Incremental validation gates

### Gate 1 — signal validity

- Correct boundary identity is visually confirmed.
- Camera timestamps remain aligned with tracking logs.
- No false sign reversal occurs across representative straights and curves.
- Availability and confidence are reported by road section.

### Gate 2 — shadow-controller validity

- Applied correction remains exactly zero for every sample.
- Candidate correction points toward increased measured clearance.
- Candidate remains within its configured bound.
- Candidate changes satisfy the slew-rate limit.
- Candidate timing leads, rather than follows, physical boundary encroachment.

### Gate 3 — low-authority single-car validation

- Begin with `.151` only at reduced speed.
- Compare at least five baseline and five hybrid trials from the same start.
- No increase in outer-boundary violations, steering saturation, localization
  faults, or route failures is allowed.
- Median and 95th-percentile physical lane-center error must improve.

### Gate 4 — cross-vehicle validation

- Repeat the accepted controller on `.152` without node-specific code.
- Confirm both route families and the roundabout.
- Permit vehicle-specific camera calibration, but keep one control algorithm.

### Gate 5 — coordinated two-car validation

- Repeat the controlled CZ1 arrival scenario.
- Measure minimum separation and confirm the waiting vehicle resumes only
  after release.
- Demonstrate that improved lane placement prevents unnecessary YOLO blocking
  when vehicles pass in neighboring lanes.

## LaneNet decision

LaneNet is a candidate segmentation source, not a controller. It should be
evaluated against the existing HSV/geometry detector using identical saved
frames and the same physical-clearance labels. Adopt it only if it materially
improves boundary availability or geometric accuracy without unacceptable
latency. The fusion and safety interfaces remain unchanged whichever detector
wins that comparison.

## Current implementation boundary

The current code provides:

- a versioned local UDP camera-guidance observation;
- a confidence-gated, bounded, slew-limited shadow candidate;
- a calibration/authority interlock;
- explicit zero applied authority;
- synchronized hybrid fields in `tracking_diagnostics.csv`; and
- status output proving candidate, authority, and applied correction.

The repository contains an opt-in bounded camera-fusion implementation, but
the normal launcher does not enable it. The latest physical steering candidate
still crossed the divider and was rejected. Earlier preventive-stop behavior
improved safety containment but did not complete the route or recover
automatically. Physical calibration and Gates 1–2 remain the
next required work.
