# Plan: Response to Supervisor Feedback (2026-11-05)

**Status (2026-11-19):** feedback reviewed and decisions agreed (below). Nothing started yet except what already sits in the [camera plan](camera_lane_correction.md). The camera report's "Where to pick up" block (drive A → drive B → fusion → left camera) is still the immediate next work; this doc adds what comes after it and what the next report needs.

**Source:** Dr. Fu's *Technical Feedback and Research Development Guidelines* (reviewing the [2026-10-30 update](../reference/SUPERVISOR_PROJECT_UPDATE_2026-10-30.txt)), plus her follow-up note on path planning and multi-vehicle coordination. The PDF is kept by the user, not in this folder.

## What she asked for (summary)

| # | Topic | Main requests |
|---|---|---|
| 1 | Steering calibration | Per-car procedure; commanded vs measured wheel angle (plot), both directions, speeds; separate Car A / Car B parameters; before/after tracking comparison; how much deviation was actuator bias vs localization/reference error |
| 2 | Localization | Coordinate-frame diagram (map, LiDAR, body, controller, camera) and transforms; error at known physical positions incl. straights, curves, intersections; raw vs corrected; positions not used to build any correction; effect of landmark changes |
| 3 | Lane boundary safety | Physical lane boundaries; footprint (wheels, body); **minimum physical clearance**; straights vs curves; speeds; count every contact |
| 4 | Localization safety | 2 s timeout ≈ 0.8 m at 0.4 m/s is too long; detection/response time; speed-dependent timeout; slow down when confidence drops; recovery tests |
| 5 | Camera correction | Calibration, detection examples, lateral accuracy vs ground truth, architecture diagram; **controlled comparison**: waypoint-only vs camera-assisted vs integrated with boundary monitoring |
| 6 | Waypoint cleanup | Pseudocode, criteria, before/after plots, complexity, limits; say honestly whether it is general or a fix for Quanser's `generate_path` |
| 7 | Repeated trials | ≥ 10 trials per car per speed: completion rate, mean/max/RMS lateral error, min clearance, localization failures, stops, lap time; mean, sd, range; estimated vs physical kept apart |
| 8 | Multi-vehicle | Scenarios A (separate routes), B (shared intersection), C (leader–follower), D (communication loss) |
| 10 | Reproducibility | GitHub with latest code; **commit/version for every reported experiment** |
| 11 | Research contribution | Pick one direction (A lane-constrained navigation, B fault tolerance, C coordination); problem, prior work, novelty, baselines, hypotheses; literature review |
| Note | **Route planning** | Routes are currently predefined (`nodeSequence` typed in `vehicle_control.py`; Quanser's `generate_path` only fills paths between the given nodes). She wants: road-network graph, A* from start to destination per car, pseudocode + code, then intersection coordination, then conflict-aware coordination and replanning |

Her priorities for the next report: **1** reproducible single-vehicle performance with independent clearance measurements, **2** camera correction with controlled comparison, **3** at least one two-car scenario, **4** documentation + research contribution. For each: implementation status, method, quantitative results, limitations, next steps.

## Decisions (agreed with the user, 2026-11-19)

- **Single vehicle first.** The full lap comes from getting camera steering + fusion right; two-car work (route planning, coordinator, scenarios) starts only once single-vehicle driving is solid. The user already has intersection-coordinator code to review at that point (not in this folder yet).
- **GitHub:** already done by the user. Still to do: record the commit used for each reported drive.
- **Kalman fusion** (camera report top block, step 3) comes **after drive A and drive B**, one change per drive. Open question: write and test it offline (`c3_replay.py` on the §40 / §46 logs) while the drives are done, without putting it on the car until both are evaluated. The user hasn't decided yet.
- **Research direction (suggested, not yet chosen):** A, robust lane-constrained navigation with confidence-weighted GPS + camera + odometry + map fusion and uncertainty-triggered stops. It covers her sections 1, 2, 4 and 5 and keeps the GPS as a core sensor (the user's direction, 2026-11-19). C is still needed as a demonstration.

## Controlled comparison: what exists and what's needed

**Existing logs are preliminary evidence only, not a controlled comparison:**

| Drive | Steering | Camera logged | Coverage | Limitation |
|---|---|---|---|---|
| Oct 28 car A, Oct 29 car B full laps | GPS + table | no | full lap | old calibration, no independent lane measurement |
| 2026-11-10 lap A (`vehicle_control_lapA_no_camera.log`) | GPS + table | no | full lap, 29 rejections | before the 11-11 recalibration |
| **2026-11-18 measure-only (§34)**, `raw_frames/20261118_222611`, `c2_20261118_222611_*.csv` | GPS + table | **yes** | 0 → 2 → 4 → start of 4 → 14, 0.2 m/s | table was stale (built for the October calibration): 8–10 cm off at the start / node 0, within ±4 cm on node 2 → node 4 |
| 2026-11-18 camera drives (`225612` … `234715`), 2026-11-19 `Fresh_Log/20261119_234304` | camera | yes | up to node 16, 0.4 m/s (last one) | the camera can't independently judge its own steering |

**Correction table: don't restore the old rows** (§43 lists them). They belong to the pre-11-11 calibration and would make the GPS baseline look worse than it is. The table only ever corrected the lower map (nodes 0–4). Instead:

1. **Baseline drive: raw GPS steering, no table** (`use_camera_steer = False`), camera measuring only (`lane_camera.py --save-raw`), at the same speed as the camera drives. The camera offset on 0 → 2 → 4 shows whether the table is still needed: if raw GPS stays within about ±4 cm, it isn't.
2. **Only if raw GPS exceeds that on the lower map:** re-survey and rebuild the table for the current calibration, and add "GPS + table" as its own configuration.
3. **Configurations to compare** (same car, speed, start, commit): GPS only, (GPS + table if needed), camera steering, fusion. Then her third one: fusion + camera boundary monitoring / safety intervention.
4. **Independent judge** (neither the steering sensor nor the front camera): a phone video from above with tape marks at a few points (node 0, mid 0 → 2, node 2, node 4, roundabout), or hand-measured stops. Later possibly the left camera.

This needs a full lap first, so it comes after the fusion work.

## Order of work

1. *(in progress, camera report)* Drive A (trim −0.04), drive B (curve speed limit), Kalman fusion, left camera if still needed → **full lap on car A**.
2. Controlled comparison (above), then repeated trials: 10 laps per configuration and speed, plus a script that produces her metrics table from the logs.
3. Measurements for the report: steering angle vs command for both cars (plot), GPS error at taped points incl. curves/intersections, frame diagram, physical clearance method.
4. Per-car settings (car A / car B), car B steering trim measured, car B on the same code.
5. Write-ups that need no car: waypoint-cleanup pseudocode/plots/complexity, frame diagram, architecture diagram.
6. **Then multi-vehicle:** road graph + A* start→destination per car, review the user's coordinator code, scenario A, then B (shared intersection), then C/D; conflict-aware coordination and replanning later.
7. Research direction chosen + literature review before writing the publication.
