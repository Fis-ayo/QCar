# Lane-Keeping Objective and Acceptance Plan

## Objective

Each QCar must complete its assigned route while its complete tire/body
footprint remains inside the designated lane. Waypoint navigation selects the
route; camera perception measures local physical-boundary clearance. A camera
safety stop is a fallback, not the final lane-keeping solution.

The project must distinguish three outcomes:

1. **Unsafe:** any tire crosses the painted divider or outer road boundary.
2. **Safe fallback:** the vehicle stops before boundary contact but cannot
   complete the route.
3. **Accepted lane keeping:** the vehicle completes repeated laps without
   boundary contact or safety intervention while maintaining the required
   physical clearance.

## Physical quantity to control

The primary quantity is minimum tire-envelope clearance to the nearest lane
boundary, not map-reference cross-track error and not the yellow line's image
offset alone.

The allowable vehicle-center error must be calculated from measured geometry:

```text
allowable center error
    = lane-center-to-boundary distance
      - vehicle half-width at the outside of the tires
      - selected safety margin
```

The operator-confirmed physical measurement gives approximately `5.0 in`
(`127 mm`) from lane center to the nearest painted-divider edge. The painted
divider is approximately `0.5 in` wide, but its center is not the calibration
reference.
Vehicle outside-tire width and the project safety margin must be measured and
recorded before setting the final numerical acceptance threshold.

## Why another arbitrary image threshold is not justified

Candidate 4 stopped at image offset approximately `-0.15`, after lane entry.
Candidate 5 stopped at approximately `-0.37`, but the operator observed that
the left front and rear wheels were already beyond the yellow divider. The
Candidate 5 controller received STOP promptly and the vehicle settled within
about `67 ms`, moving less than `1 mm` after the logged stop sample. Therefore,
the remaining error is not primarily braking or communication delay.

Image offset depends on camera placement, perspective, vehicle heading, curve
geometry, and vehicle footprint. It must be related to measured tire clearance
before it is used as a physical boundary threshold.

## Next experiment: stationary clearance calibration

This does not require a checkerboard or camera intrinsic calibration.

1. Select representative straight, moderate-curve, and tight-curve locations.
2. At each location, align the QCar with its intended direction of travel.
3. Measure the vehicle centerline relative to the yellow divider using a ruler;
   also record nearest-tire clearance when evaluating boundary safety.
4. Capture stationary CSI data at these conditions:
   - centered in lane;
   - chosen safe clearance;
   - tire edge at the divider boundary;
   - small controlled intrusion, without driving autonomously.
5. Record location, vehicle, heading, physical clearance, yellow offset,
   yellow heading, fitted curvature, confidence, and saved frame.
6. Repeat on both QCars and on both left- and right-curving examples where the
   relevant boundary is visible.

The resulting dataset will determine whether one conservative feature rule is
valid across routes and vehicles. If it is not, the next perception model must
estimate lane geometry/vehicle position rather than use one offset threshold.

## Staged acceptance gates

### Gate A — Boundary measurement

- The correct physical divider is identified in every evaluated frame.
- A conservative clearance estimate or classifier separates safe clearance
  from tire contact across held-out locations and both vehicles.
- False-safe events are not allowed in the validation set.

### Gate B — Safety fallback

- In five consecutive supervised trials per vehicle, the safety system stops
  before any tire reaches the divider.
- Communication loss still produces a fail-closed stop.
- Stop decision, speed response, and minimum clearance are logged.

### Gate C — Lane-centering control

- Five consecutive complete laps per vehicle with zero divider/outer-boundary
  contacts.
- No manual stop and no critical camera safety stop.
- Minimum physical clearance remains above the selected safety margin.
- Straight and curved results are reported separately.
- Map-reference error and physical-boundary clearance are both reported; one
  must not be substituted for the other.

### Gate D — Two-vehicle readiness

- Both vehicles pass Gate C individually.
- Opposing/adjacent-lane passing and CZ1 interaction maintain positive minimum
  separation without YOLO/reservation deadlock caused by lane intrusion.

## Decision rule for further controller work

Do not change Stanley gain, tracking offset, camera threshold, or steering
fusion simultaneously. Every candidate must state:

- the physical failure it addresses;
- the single principal change;
- its predicted effect;
- the fixed comparison configuration; and
- the acceptance gate it is intended to pass.

Camera steering fusion should begin only after Gate A establishes a reliable
direction and clearance signal. The first steering candidate must be bounded,
must preserve waypoint route selection, and must retain Candidate safety logic
as an independent final restriction.

## Current gate status — October 5, 2026

`hybrid_safety_2` visually confirmed one `.151` stop with the entire vehicle
inside its lane. This is meaningful safety progress but does not satisfy the
five-trial, two-vehicle Gate B requirement. Automatic recovery was not
implemented, and the vehicle did not complete a lap.

The latest bounded steering candidate (`hybrid_final_1`) stayed within its
software correction limit and pointed away from the divider, but the vehicle
still crossed. It is rejected and disabled. Another physical steering run is
not justified without a materially improved continuous lane/corridor estimate.
