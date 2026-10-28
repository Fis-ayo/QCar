# Plan: Camera Lane Correction

**Status:** planned, not started. From the [new-route record](../history/2026-10-28_new_route_investigation.md) §15.

## Why

The GPS correction table ([car_A_setup.md](../car_A_setup.md) §1) treats the symptom: it adds back an error measured once in the lidar readings. It is only as good as those measurements and is invalid after any room or calibration change. Walls near the lower part of the map would make the lidar position more accurate, but have the same dependence on the room. Even at its best, the lidar position (±2–3 cm) uses most of the ~4 cm clearance.

**Camera lane correction measures the thing that matters:** the car's sideways position between the painted lines, independent of the lidar localization.

| | Correction table (now) | Walls + recalibrate | Camera correction |
|---|---|---|---|
| What it measures | Nothing new: adds back an error measured once | Better lidar position | The car's actual place in the lane |
| Survives room or calibration changes | No | No | Yes |
| Works on car B without retuning | No | No | Mostly (same camera mounting) |
| Effort | Done | Moderate | Most |

## What it does and doesn't replace

**It does not replace the GPS.** The lidar position is still needed to know where the car is on the map, when to turn and which route to follow. The camera only corrects the sideways position within the lane.

**Keep the correction table until the camera is proven.** At intersections, or wherever the lines aren't visible, the car falls back to GPS steering, and the table keeps that fallback accurate on the lower part.

## How it would fit the current code

- **Where it runs:** [qcar/yolo_server.py](../../qcar/yolo_server.py) already reads every RealSense frame. It can also detect the lane lines and send the sideways offset to `vehicle_control.py`, the same way it sends detections now, so there is no second process competing for the camera.
- **Detection:** start simple. Find the yellow divider by colour, then convert the image to a top-down view to get the offset in cm (`LaneKeeping` in `hal.content.qcar_functions` already has that camera geometry). [skills_activities/06-lane_keeping/](../../skills_activities/06-lane_keeping/) shows the steps, but it is written for QLabs with keyboard driving and needs adapting.
- **Controller:** when the line is detected confidently, the camera offset replaces the GPS cross-track error in the Stanley term. Otherwise the controller falls back to GPS (with the correction table) as now.

## Plan (measure first, then steer)

1. **Measure only:** add yellow-line detection to the YOLO server and log the measured offset during a normal drive. The car's behaviour doesn't change.
2. **Check it:** compare the logged offsets with what the car visibly does, especially on the 0 → 2 and 2 → 4 curves.
3. **Use it in the steering,** with GPS as the fallback, once the measurement holds up. Then decide whether the correction table is still needed.
