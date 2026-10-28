# Car A: New Route Investigation (`0 → 2 → 4 → 14 → 16 → 18 → 12 → 0`)

Car A, tuned on the loop `0 → 2 → 4 → 6 → 0` ([car_A_single_vehicle_setup.md](car_A_single_vehicle_setup.md)), was run on a longer route. It drove 0 → 2 → 4 fine, then went wrong just after entering node 14's lane (the far-right lane, x ≈ 2.25, going up the map).

**Status (2026-10-28): the new route works.** Drive 6 completed the full route at `v_ref = 0.2`, with the car in its lane ("good enough for now"; further fine-tuning is possible).

What it took:
1. **Localization:** walls near node 14 had been moved after calibration, so the lidar confused the upper right lane with other places on the map. Fixed by putting the walls back, adding landmarks near node 14 and by the top road, and recalibrating (§4–§8). **Keep the walls and both landmarks where they are; any change needs a recalibration.**
2. **Safety:** a GPS-jump check ignores impossible readings and stops the car if none are accepted for 2 s (§9).
3. **Path bug:** a backward step in the generated path at node 16 made the car miss node 18. `remove_backward_points()` fixes it for any route (§11).
4. **Lower part of the map:** it has no walls nearby, and the localization there is off by several cm. A position-dependent GPS correction table compensates for it, tuned over drives 4–6 (§10, §12, §13). `gps_scale` is 1.0. **The table is only valid for the current calibration.**

Final settings are in §14. **Planned next step:** camera lane correction (§15), the long-term replacement for the correction table.

> **Log files:** `logs/vehicle_control.log` is overwritten by each drive and now holds **drive 6**. Line references in §1, §9 and drives 1–5 refer to earlier versions of that file that are no longer on disk.

---

## 1. What the log showed

Log: [logs/vehicle_control.log](../logs/vehicle_control.log) (route `[0, 2, 4, 14, 16, 18, 12, 0]`, `v_ref = 0.4`).

- **Tracking was normal up to y ≈ 1.5.** `cross_track` was 0.00 to 0.05 m through 0 → 2, 2 → 4 and the start of 4 → 14, the same as on the old loop.
- **Then the lidar position fix jumped** ([line 118 → 119](../logs/vehicle_control.log#L118)): `(2.30, 1.48)` → `(1.30, 1.58)`, a 1.00 m jump in one reading. After that it kept switching between unrelated spots: `(2.00, 2.97)`, `(-0.38, 3.59)`, `(-0.08, 1.32, -117°)` (the x = 0 street, facing the opposite way), `(1.04, 4.37)`, …
- **The controller followed every bad reading.** The EKF has no plausibility check, so its estimate jumped too, `cross_track` reached −0.97 m and then −2.4 m, and steering saturated. Those were the "untracked turns".
- **The route progress got corrupted too.** The controller's waypoint index only moves forward, so while the estimate was jumping it advanced into the 14 → 16 and 16 → 18 segments.

- **The car made the route's later turns in the wrong place.** In [lines 159–200](../logs/vehicle_control.log#L159) the estimate traces a smooth 16 → 18 → 12 path ((1.23, 3.66, 146°) → (0.54, 3.41, −137°) → (−0.06, 2.23, −96°)), and the controller followed it with `cross_track` of a few cm. The user saw the car make the intended movements, but early, while still in node 14's lane. The controller was doing its job; it was working from a wrong position.

The old loop turns left at y ≈ 1.1, so car A had never driven x = 2.25, y > 1.2 before this run.

## 2. Causes ruled out

| Suspected cause | Verdict |
|---|---|
| `gps_scale` used outside the area where it was measured | Not the cause: it shifts the estimate by centimetres, and the jumps were about 1 m |
| Stanley steering blowing up at low speed (YOLO slowdowns) | No sign of it: GPS readings were evenly spaced (steady speed) and `cross_track` was small until the jump |
| Less right-turn range because of `steering_trim` | Never reached: the car didn't get to the right turn at 16 → 18 |
| **Lidar localization jump** | **Confirmed.** Same kind of dropout `check_pose.py` hit twice before (setup doc §6), now while driving |

No code changes were made for these.

## 3. Next step: localization survey

**Question:** does the position fix jump or get lost along the x = 2.25 lane even with the car standing still?
- **Yes, at certain spots:** the lidar can't get a reliable fix on that stretch. Look at what the lidar sees from there (open space, people, symmetric walls) or recalibrate.
- **No, stable everywhere:** the problem only happens while driving. Next is the GPS-jump safety check (setup doc next step 3), which makes the car reject impossible readings or stop instead of following them.

[qcar/check_pose.py](../qcar/check_pose.py) already has `nodeSequence = [0, 2, 4, 14, 16, 18, 12, 0]`.

### Where to place the car

All spots are in the **lane on the far right edge of the mat** (x ≈ 2.25, from node 4 up to node 14). The car is **centred in the lane, pointing up the map** (the direction it drives toward 14). Measure the distance with a tape **from node 4's marker** along the lane.

| # | Distance past node 4 | Map y | How to find it |
|---|---|---|---|
| 1 | 0.4 m | ≈ 0.5 | Just past node 4; the old route drove here (known-good reference) |
| 2 | 0.9 m | ≈ 1.0 | About where the old route turned left toward node 6 |
| 3 | 1.2 m | ≈ 1.3 | New ground; the log first wandered here (`cross_track` 5 cm) |
| 4 | 1.45 m | ≈ 1.55 | **Where the position jumped in the failed run** |
| 5 | 1.8 m | ≈ 1.85 | Level with node 15 (in the lane beside it) |
| 6 | 2.1 m | ≈ 2.2 | Between node 15 and node 14 |
| 7 | 2.9 m | ≈ 3.0 | On node 14 |

At each spot:
1. Wait for `n=25/25` (about 5 s; longer if the reading is unstable).
2. Watch for another 5 s. Note the spot number if `GPS=` shows a place that isn't where the car is, or `n` drops back to a small number while the car is still. A small `n` means the reading jumped more than 10 cm.
3. **Roll** (don't lift) the car to the next spot.

Stay out of the lidar's view (below about car-roof height) while each spot settles. If a spot jumps, roll back to it once to see whether it happens again.

### Commands

On the PC, from the project folder:
```
scp qcar/check_pose.py nvidia@192.168.0.151:/home/nvidia/Documents/self_driving/multi_vehicle_self_driving/
ssh nvidia@192.168.0.151
```
On the car (SSH):
```
cd /home/nvidia/Documents/self_driving/multi_vehicle_self_driving
python -u check_pose.py | tee -i check_pose.log
```
Press **Ctrl+C** after the last spot (`-i` stops `tee` from quitting on Ctrl+C, so the last lines are kept), then `exit`.

Back on the PC:
```
scp nvidia@192.168.0.151:/home/nvidia/Documents/self_driving/multi_vehicle_self_driving/check_pose.log logs/check_pose/check_pose_round3_new_route.log
```

## 4. Results

Log: [logs/check_pose/check_pose_round3_new_route.log](../logs/check_pose/check_pose_round3_new_route.log). The car was held still, centred in the x = 2.25 lane and pointing up the map (+90°). GPS values are raw (`gps_scale` not applied).

| Spot | Log lines | What the GPS reported | Verdict |
|---|---|---|---|
| 1 (0.4 m) | 2–49 | (2.27, 0.84, 90°), offset −0.026 m, stable | Correct lane and heading |
| 2 (0.9 m) | 74–119 | (2.27, 1.38, 90°), offset −0.011 m, stable | Correct lane and heading |
| 3 (1.2 m) | 120–223 | Off the mat at (3.6–3.9, −2.3…1.3), heading ≈ −85°; never settled | **Wrong** |
| 3, re-aligned | 224–318 | Briefly (1.28, 1.68, 93°) (x 1 m off, as in the driving run), then **stable** at (0.10, 1.72, −89°) | **Wrong: other side of the map, facing the opposite way** |
| 4 (1.45 m) | 327–442 | (0.13, 1.44, −88°); repeated resets from single outlier readings | **Wrong (mirror)** |
| 5 (1.8 m) | 475–535 | (0.13, 1.10, −88°); outliers to (−1.4, 2.4) | **Wrong (mirror)** |
| 6 (2.1 m) | 578–616 | (0.16, 0.78, −88°) | **Wrong (mirror)** |
| 7 (2.9 m) | 661–762 | (−2.63, 0.53, 2°), stable; off the mat, rotated 90° | **Wrong** |

**Findings**

1. **Localization fails from about 1.2 m past node 4 onward, with the car standing still.** Below that (spots 1–2) it is correct to about 1–3 cm sideways.
2. **Mostly it locks onto one consistent wrong answer:** the lane at x ≈ 0.1, facing down the map. That is the car's real pose rotated 180° about roughly (1.2, 1.5). The wrong y values line up with this exactly: true y 1.3 / 1.55 / 1.85 / 2.2 read as 1.72 / 1.44 / 1.10 / 0.78, and each pair adds up to about 3.0. This is the same wrong pose the failed driving run jumped to ((−0.08, 1.32, −117°)).
3. **The wrong answer is stable,** with spread under 1 cm. The localization is confident: from this lane the lidar scan matches the mirrored spot about as well as the true one.
4. The "n restarting from 1" you saw was single outlier readings more than 10 cm away resetting the window (for example (0.06, 2.76) at line 319 and (−1.4, 2.4) at lines 446–474).

**Conclusion:** the new route fails because the lidar localization can't tell the upper right lane apart from its mirror image on the other side of the map. This is a localization/environment problem, not a tuning one. A GPS-jump safety check would stop the car here but wouldn't let it drive this lane, because the readings stay wrong rather than briefly jumping.

**Likely cause:** white walls stood along the sides near node 14 when the car was calibrated, and they have since been moved. The localization compares lidar scans with what it recorded at calibration, so near node 14 the recorded walls are missing from the scans. In a rectangular room the best remaining match is then the mirrored pose. The lower part of the lane (spots 1–2) is far enough from where the walls stood to still match correctly.

**Next test:** either put the walls back where they were and repeat spots 3–7 (no recalibration needed), or recalibrate in the current layout and repeat the full survey. After a recalibration, also recheck `gps_scale` on the old loop, because it was measured with the old calibration.

**Side note:** spots 1 and 2 read y ≈ 0.84 and 1.38, about 0.4 m further up than the tape distances suggest. The sideways offsets were fine. About 0.2 m of this is because the **rear** of the car was placed on each mark, so the car's middle (the lidar) was about 0.2 m further up. The remaining 0.15–0.2 m is unexplained; it could be where the node 4 marker sits on the mat relative to the map coordinate. It doesn't affect lane keeping and doesn't change the mirror finding (the paired y values still add up to a constant). In later surveys, put the **middle of the car** on each mark.

## 5. Round 4: walls put back

Log: [logs/check_pose/check_pose_round4_walls_back.log](../logs/check_pose/check_pose_round4_walls_back.log). All 7 spots, **middle** of the car on each mark, rolled between spots. GPS values are raw.

| Spot | Expected y | Log lines | Reported | Verdict |
|---|---|---|---|---|
| 1 (0.4 m) | ≈ 0.48 | 2–81 | (2.27, 0.65, 90°) | ✅ correct lane and heading; y +0.17 |
| 2 (0.9 m) | ≈ 0.98 | 111–197 | (2.25, 1.18, 92°) | ✅; y +0.20 |
| 3 (1.2 m) | ≈ 1.28 | 215–294 | (2.25, 1.48, 92°) | ✅; y +0.20 (was the mirror in round 3) |
| 4 (1.45 m) | ≈ 1.53 | 315–471 | (2.21, 1.76, 91°) | ✅; y +0.23 (was the mirror) |
| 5–7 (1.8 / 2.1 / 2.9 m) | ≈ 1.88 / 2.18 / 2.98 | 472–1004 | (2.30–2.34, **−0.1 … 0.25**, 92°) | ❌ correct lane and heading, but **2–3 m wrong along the lane** |

**Findings**

1. **Putting the walls back removed the mirror.** Every reading is in the right lane with the right heading, and there are no off-mat poses. That confirms the moved walls caused the round 3 failure.
2. **Spots 1–4 read a consistent +0.2 m along the lane.** This is the same unexplained offset as round 3, so it's a fixed offset (most likely the node 4 marker vs. the map coordinate), not a localization fault.
3. **From spot 5 on, the y position jumps to around 0.0–0.25,** which is near node 4, while the car is 2–3 m further up. The jump happened in one step at line 472. A long straight lane alongside a wall looks similar at every point along it, so the scan matcher can slide along it. The walls probably aren't exactly where they were at calibration, which leaves the match weakest at the top of the lane.
4. **Sideways readings also changed with the wall layout.** At spots 1–4 the raw x is already 2.21–2.27 (close to the lane's 2.25). Applying `gps_scale = 1.035` would put the estimate 7–9 cm toward the divider. So `gps_scale` has to be re-measured after recalibrating.

**Next:** recalibrate with the walls where they are now, repeat the full survey (round 5), and recheck `gps_scale` on the old loop. If spots 5–7 still slide along the lane, place something distinctive (for example a box) beside the upper part of the lane, near node 14, then recalibrate again.

## 6. Round 5: recalibrated, spots 5–7

Log: [logs/check_pose/check_pose_round5_recalibrated.log](../logs/check_pose/check_pose_round5_recalibrated.log). Recalibrated with the walls in their current place; only spots 5–7 surveyed. Expected y includes the fixed +0.2 m marker offset from round 4.

| Spot | Expected y | Log lines | Reported | Verdict |
|---|---|---|---|---|
| 5 (1.8 m) | ≈ 2.08 | 5–46 | (2.29, 2.09, 89°), offset −0.035 m | ✅ (one stray reading at (−0.84, 2.23, −91°) when placed) |
| 6 (2.1 m) | ≈ 2.38 | 52–118 | (2.29, 2.41, 90°), offset −0.039 m | ✅ |
| 7 (2.9 m, node 14) | ≈ 3.18 | 124–236 | **(−0.83, 1.66 … 2.3, −91°)**, stable | ❌ wrong place, ~3 m to the left and facing down the map |

**Findings**

1. **Recalibrating fixed spots 5 and 6.** The along-lane slip from round 4 is gone there, which confirms it came from the old calibration no longer matching the walls.
2. **Node 14 still locks onto a wrong pose,** about (−0.83, 1.7–2.3) facing down the map, and stays there. The same wrong pose showed up once as a stray reading at spot 5, so it is a known wrong match for this part of the lane.
3. **Sideways** (raw): x ≈ 2.29 in the 2.25 lane at spots 5–6. `gps_scale` still needs re-measuring before driving.
4. **The rest of the route has not been surveyed yet:** 14 → 16 (top-right corner and top of the circle), 16, 18, 18 → 12 and 12. These need checking too.

**Next:** put something distinctive near node 14 (for example a box against the wall at the top-right, off the road), recalibrate, then survey node 14 and the rest of the route.

## 7. Round 6: landmark near node 14, recalibrated, node 14 onward

Log: [logs/check_pose/check_pose_round6_landmark.log](../logs/check_pose/check_pose_round6_landmark.log). Landmark placed near node 14, then recalibrated. Spots A–G from node 14 to node 12, using the node markers.

| Spot | Log lines | Reported | Expected | Verdict |
|---|---|---|---|---|
| A node 14 | 5–48 | (2.25, 3.05, 90°), offset +0.001 m | (2.25, 2.97, 90°) | ✅ (was wrong in round 5) |
| B top-right corner | 76–123 | (2.24, 3.81, 93°), offset +0.019 m | lane at x 2.25, bend starts ≈ 3.9 | ✅ |
| C top road | 124–274 | never correct; stable wrong poses (2.91, 0.05, −87°) at 150–193 and (−0.67, 3.64, −65°) at 218–262, plus scattered ones | ≈ (1.5, 4.49, 180°) | ❌ |
| D node 16 | 275–467 | (0.84–0.91, 3.74, −82°), offset −0.063 m | (0.91, 3.71, −81°) | ✅ (6 cm right of path) |
| E node 18 | 486–584 | (0.64, 3.06, −139°), offset +0.017 m | (0.62, 3.07, −138°) | ✅ |
| F mid 18 → 12 | 587–765 | (0.13–0.28, 2.6–2.7, −130°), offset +0.002 m | on the diagonal | ✅ |
| G node 12 | 766–800 | (−0.03, 1.69, −90°), offset −0.026 m | (0.00, 1.85, −90°) | ✅ |

**Findings**

1. **The landmark fixed node 14.** A and B are now correct to about 2 cm sideways.
2. **D to G (node 16 to node 12) are all correct,** with the heading within a few degrees.
3. **Only the top road (C) still fails.** There, the localization never found the right pose. One of the wrong poses, (2.91, ~0, −87°), is off the mat to the right and facing down the map. Like node 14 before the landmark, the area along the top edge seems to look like somewhere else to the lidar.
4. Readings on the move between A and B jumped along the lane (y 2.94 ↔ 3.87, lines 49–75); this needs watching once driving.

**Next:** confirm where the car was during lines 124–274, then fix C (a second landmark along the top edge, then recalibrate) and resurvey C alone.

## 8. Round 7: second landmark by the top road, recalibrated

Log: [logs/check_pose/check_pose_round7_top_road.log](../logs/check_pose/check_pose_round7_top_road.log). A second landmark was placed beside the top road, then the car was recalibrated. Spots B, C and D, rolled between them.

| Spot | Log lines | Reported | Expected (path) | Verdict |
|---|---|---|---|---|
| B top-right corner | 2–52 | (2.19, 3.98, 103°), offset +0.04 m | (2.23, 3.98, 105°) | ✅ |
| Rolling B → C | 53–56 | (2.07, 4.16, 127°) → (1.72, 4.42, 149°) → (1.43, 4.50, 168°) → (1.32, 4.43, −168°) | follows the path | ✅ tracked while moving |
| C top road / start of the curve into the circle | 57–138 | (1.25–1.31, 4.38–4.44, −151 … −159°), offset within ±0.03 m | (1.27, 4.42, −153°) | ✅ (one stray reading at line 74) |
| Rolling C → D | 139–141 | stray (0.93, 4.63) at line 139, then (0.92, 3.86) → (0.90, 3.72) | follows the path | ✅ |
| D node 16 | 142–180 | (0.86–0.92, 3.72–3.76, −84°), offset −0.01 … −0.05 m | (0.90, 3.76, −85°) | ✅ |

**Findings**

1. **The second landmark fixed the top road.** C reads correctly and stays stable.
2. **The position also tracked correctly while the car was rolled** from B to C to D. This is the first time the moving readings have followed the path through this area, a good sign for driving.
3. **Two single stray readings remain** (lines 74 and 139), each followed straight away by correct ones. The GPS-jump check is meant to catch exactly this.
4. Combined with round 6, every surveyed spot on the new route now reads correctly. Spots 1–6 on the 4 → 14 lane were last surveyed before the two landmarks and the latest recalibration.

**Next:** add the GPS-jump check to `vehicle_control.py`, re-measure `gps_scale`, then a slow test drive (`v_ref = 0.2`).

## 9. GPS-jump check (added to `vehicle_control.py`)

Settings, next to `gps_scale` in [qcar/vehicle_control.py](../qcar/vehicle_control.py):

| Setting | Value | What it does |
|---|---|---|
| `gps_max_jump` | `0.3` m | A GPS reading further than this from the EKF estimate is ignored |
| `gps_max_heading_jump` | 30° | Same, for heading |
| `gps_timeout` | `2.0` s | If no reading has been accepted for this long, the car stops and the script ends |

- **Ignored readings:** the EKF continues on wheel odometry and the gyro, and the log shows `GPS rejected: (x,y,heading) jump=… heading_jump=…`. Accepted readings still print as `GPS: …`.
- **Stop:** the log shows `STOPPED: no GPS reading accepted for 2 s. Last estimate (…)`. This happens when the readings stay wrong, or when the estimate itself has drifted so far that correct readings are rejected. Either way, stopping is the safe outcome; check the log and restart.
- **EKF start:** the EKF now starts from the first GPS reading instead of node 0's pose. The car starts about 30 cm before node 0, so otherwise the check could reject the first readings.

**Replay check:** running the failed drive's GPS readings ([logs/vehicle_control.log](../logs/vehicle_control.log)) through the same rule gives no rejections before the first jump. The jump at line 119 (1.00 m) and every reading after it are rejected, and the car would have stopped at about line 132, about 2 s after the first bad reading, instead of steering toward the wrong positions.

## 10. Round 8: `gps_scale` survey (current calibration)

Log: [logs/check_pose/check_pose_round8_scale.log](../logs/check_pose/check_pose_round8_scale.log). Order: node 0, node 2, halfway round 2 → 4, halfway up 4 → 14 (unstable), node 18, node 12, halfway up 4 → 14 again (stable).

**Trusted readings** (from rounds 7 and 8, current calibration). Sideways error = raw GPS minus lane centre, measured across the lane. The last column shows what dividing by 1.035 would give.

| Spot | Log lines | Raw GPS | Map | Sideways error, `gps_scale = 1.0` | Error with 1.035 |
|---|---|---|---|---|---|
| Node 0 (x = 0 lane) | 2–34 | (0.05, −0.07, −90°) | x = 0.00 | +5 cm | +5 cm |
| Node 2 (bottom lane) | 39–43 | (1.16, −1.12, −1°) | y = −1.08 | 4 cm outward | 0 cm |
| Halfway up 4 → 14 (2nd try) | 498–523 | (2.30, 1.92, 90°) | x = 2.25 | 5 cm outward | −3 cm |
| Node 12 (x = 0 lane) | 411–456 | (−0.04, 1.70, −90°) | x = 0.00 | 4 cm outward | 4 cm |
| B top-right corner (round 7) | | (2.19, 3.98) | x = 2.23 | −4 cm | −11 cm |
| C top road (round 7) | | (1.27, 4.43) | (1.27, 4.42) | 0 cm | ≈ 14 cm |
| D node 16 (round 7) | | (0.88, 3.75) | x = 0.90 | −2 cm | −5 cm |

**Findings**

1. **`gps_scale` should go back to 1.0 (no correction).** With 1.0, every trusted spot is within ±5 cm. With 1.035 the bottom lane is slightly better, but the top of the map is 11–14 cm off, because the correction grows with distance from the origin. The errors don't follow a scale pattern; they look like ±5 cm local noise.
2. **The localization was lost for a long stretch:** from line 44 (just after node 2) to about line 408. It covered the 2 → 4 curve, the first 4 → 14 attempt and node 18. The first wrong reading, (1.21, 2.19, 7°), had about the right heading but was 3.3 m off. After that it stayed on wrong poses, mostly (−0.85, 1.2…2.5, −92°).
3. **It recovered by itself** on the way from node 18 to node 12. After that it tracked correctly while the car was rolled from node 12 along the y ≈ 0.9 road to the 4 → 14 lane. A wrong patch appeared again around lines 468–497, and then the 4 → 14 lane read correctly.
4. With the GPS-jump check, a loss like line 44 during a drive would stop the car about 2 s later instead of steering it off course. The route can't be completed until the loss itself stops happening.

**Open question:** was the car lifted or rolled between node 2 and the 2 → 4 curve (line 44)? A lifted car can lose the localization; a rolled one shouldn't.

## 11. First test drive on the new route

Settings changed in [qcar/vehicle_control.py](../qcar/vehicle_control.py) for this drive: `gps_scale = 1.0` (was 1.035, §10) and `v_ref = 0.2` (was 0.4). The GPS-jump check (§9) is active. Unchanged: `K_stanley = 1.5`, `steering_trim = -0.14`, tracked point 0.1 m.

What to look for in `vehicle_control.log`:
- `GPS rejected:` lines: where they happen and how many in a row
- `STOPPED:` line: where the car was when it stopped
- `cross_track` on each segment, especially 14 → 16 (corner and top road) and the first right turn, 16 → 18

### Drive 1 result (log: [logs/vehicle_control.log](../logs/vehicle_control.log))

- **The car started past node 0,** with a first GPS reading of (0.06, −0.07). Node 0 is at (0, 0.13). The roadmap's `initial_check` only starts the route if the car is within 0.1 m of node 0 (Dubins distance), so it drove an approach path first: 0 → 2 → 4 → 6 → 0 (the old loop). This is the same thing as setup doc §3.2.
- **The approach lap tracked cleanly:** 181 GPS readings, **0 rejected**, through the 0 → 2 curve, the bottom lane, the 2 → 4 curve, up to y ≈ 0.5 on the right lane and 6 → 0. This includes the area where the round 8 survey lost its position, so that loss was probably caused by handling the car rather than by the calibration.
- **The route itself started** at about (0.03, 0.40) (`pose=` lines from line 188), and the log ends 1.5 s later. No `STOPPED:` line, so the run was ended from outside.
- **Placement note:** the GPS reads about 0.2 m further along the lane than the mat marker. Place the car about **50 cm before the node 0 marker**, and check that the first `GPS:` line reads y ≈ 0.3–0.5.

### Drive 2 result (log: [logs/vehicle_control.log](../logs/vehicle_control.log))

Start placement correct (first GPS (0.03, 0.53)), so the route started straight away. **0 GPS readings rejected.**

| Segment | `cross_track` | What the car did (user) |
|---|---|---|
| 0 → 2 | 0.00 … 0.04 m | in lane |
| 2 → 4 | −0.01 … 0.03 m | **crossed the divider on the curve**, with part of the car still in node 4's lane |
| 4 → 14 | −0.03 … 0.04 m | in lane |
| 14 → 16 (corner, top road, circle) | −0.01 … 0.05 m | in lane, all the way to node 16 |
| 16 → 18 | path point runs off in a straight line | **did not turn into node 18's lane**; drove past, then turned left |

**Node 16: a path bug, not localization.** `generate_path` ends the 14 → 16 segment at (0.909, 3.704), 6 mm *past* node 16. The 16 → 18 segment then starts at the node itself, (0.908, 3.710). That leaves one 6 mm segment pointing **backwards** (heading +99° instead of −81°). `SteeringController` only moves to the next segment once the car is past the end of the current one, which can never happen on a backward segment. So it stayed on it and followed its extension in a straight line (logged `path_pt` from (0.91, 3.67) to (1.66, −0.81), heading ≈ −80°). The GPS readings show the car really drove that line, so the localization was correct.

**Fix:** `remove_backward_points()` in [qcar/vehicle_control.py](../qcar/vehicle_control.py) drops any waypoint that steps backwards along the path. It is applied to the route and to the approach path. It removes exactly 1 point on the new route and 0 on the old loop. In a simulation of the same controller (kinematic car, `v_ref = 0.2`), the car on the unfixed path missed node 18 by 0.30 m and went up to 0.94 m off the route; with the fix it stayed within 0.03 m through 16 → 18 → 12.

**Not specific to node 16:** the function was run on the generated path between every pair of nodes on the map, to check that it works for any node sequence:

| | Result |
|---|---|
| Node pairs with a path | 552 |
| Paths containing a backward step | 38 (about 1 in 15) |
| Backward steps left after `remove_backward_points()` | 0 |
| Largest distance of a removed point from the kept path | 4.2 mm |

The old loop `0 → 2 → 4 → 6 → 0` happens to have none, which is why this never showed up before. The function only removes millimetre-scale overshoots; it would not correct a genuinely wrong path (none of the 552 has one).

**2 → 4 divider crossing:** the controller kept its estimate on the path (`cross_track` ≤ 3 cm), so this comes from the localization. With `gps_scale = 1.0`, the bottom and right lanes read about 4–5 cm too far outward (round 8), which puts the real car 4–5 cm toward the divider, on top of the body cutting the inside of the curve. Still to be addressed.

### Drive 3 result (log: [logs/vehicle_control.log](../logs/vehicle_control.log))

Same settings as drive 2, plus `remove_backward_points()`. **Full lap of the new route completed** and the second lap started (the log ends early in the second 0 → 2). **0 GPS readings rejected, no stop.**

| Segment | `cross_track` range | Mean | Mean \|error\| |
|---|---|---|---|
| 0 → 2 | −0.013 … +0.030 m | +0.013 | 0.015 |
| 2 → 4 | −0.013 … +0.038 m | +0.012 | 0.015 |
| 4 → 14 | −0.020 … +0.035 m | +0.011 | 0.016 |
| 14 → 16 | −0.011 … +0.053 m | +0.020 | 0.021 |
| 16 → 18 | −0.017 … +0.032 m | 0.000 | 0.013 |
| 18 → 12 | −0.003 … +0.039 m | +0.013 | 0.014 |
| 12 → 0 | −0.018 … +0.032 m | +0.013 | 0.019 |

- **Node 16 → 18 now works:** the path point follows the turn ((0.91, 3.57) → (0.83, 3.29) → (0.64, 3.09) at node 18) instead of running off in a straight line as in drive 2.
- `cross_track` describes the estimate only (setup doc §3.1). Whether the car stayed in its lane has to come from watching the car.
- **What the car did (user):** from node 0 to node 4 it ran close to the divider and **crossed it visibly on the 2 → 4 curve**, as in drive 2. From node 14 to node 12 it stayed **completely in its lane**.

**2 → 4: why the lower part differs.** The estimate stayed on the path (mean `cross_track` +1 cm), so the real car was off by however far the localization is off there. Round 8 measured the bottom lane (node 2) reading 4 cm outward and the 4 → 14 lane 5 cm outward. Reading outward makes the controller move the real car the same amount **toward the divider**. Add the body cutting the inside of the left-hand curve, and the 4 cm clearance is used up. In the upper part the measured errors are mixed in sign (−4 … +4 cm), which is consistent with it staying in lane.

A single `gps_scale` cannot fix both: 1.035 corrects the lower part but moves the top of the map 11–14 cm (§10). The correction has to depend on where the car is.

## 12. Round 9: lower part survey

Log: [logs/check_pose/check_pose_round9_lower.log](../logs/check_pose/check_pose_round9_lower.log). Middle of the car on each mark, rolled between spots. There are no walls near the lower part of the mat. `side_offset`: **+ = reading is left of the path (toward the divider), − = right of the path (toward the outer edge)**. The controller moves the real car the opposite way, so a negative reading puts the real car toward the divider.

| Spot | Log lines | Raw GPS | `side_offset` mean | Range at rest | Effect on the real car |
|---|---|---|---|---|---|
| Node 0 | 9–33 | (0.06, −0.06) | +5.8 cm | +5.5 … +6.1 | 6 cm toward the outer edge |
| Mid 0 → 2 | 44–72 | (0.27, −0.81) | +1.8 cm | +1.2 … +2.3 | 2 cm toward the outer edge |
| Node 2 | 84–123 | (1.26, −1.15) | −6.3 cm | −8.0 … −4.5 | **6 cm toward the divider** |
| Mid 2 → 4 | 138–183 | (2.04, −0.88) | −7.3 cm | −10.7 … −5.3 | **7 cm toward the divider** |
| Node 4 | 195–229 | (2.32, 0.00) | −6.1 cm | −9.0 … −4.7 | **6 cm toward the divider** |
| Mid 4 → 14 | 241–264 | (2.29, 1.43) | −3.6 cm | −5.3 … −2.1 | 4 cm toward the divider |

**Findings**

1. **This explains the 2 → 4 crossing.** From node 2 to node 4 the localization reads 6–7 cm toward the outer edge, so the controller puts the real car 6–7 cm toward the divider. That is more than the ~4 cm clearance, before adding the body cutting the inside of the curve.
2. **The offset has a consistent sign at each spot,** but it wanders by ±2–3 cm over 30–60 s with the car standing still (e.g. node 2 from −4.5 to −8.0 cm). The upper part, with walls and landmarks nearby, was steadier (round 7: about ±1–2 cm). This fits the idea that the lower part has fewer nearby features for the lidar to match against.
3. **Node 0 reads 6 cm the other way.** The error is not a single shift or scale; it changes from place to place.
4. Single stray readings appeared while the car was rolled between spots (lines 124–129, 184–185), e.g. (1.62, 0.14) and (2.92, 0.28). The GPS-jump check rejects readings like these while driving.

## 13. GPS correction table (added to `vehicle_control.py`)

`gps_corrections` and `correct_gps()` in [qcar/vehicle_control.py](../qcar/vehicle_control.py) replace the plain `gps_scale` division in both places it was used (start-position check and EKF update). `gps_scale` stays at 1.0 and is still applied first.

- **Table:** one row per round 9 spot, `(x, y, dx, dy)`. The correction is the measured sideways error, turned into an x/y shift across the lane.

  | Row | x, y | dx, dy |
  |---|---|---|
  | node 0 | 0.06, −0.06 | −0.058, 0.000 |
  | mid 0 → 2 | 0.27, −0.81 | −0.013, −0.013 |
  | node 2 | 1.26, −1.15 | 0.000, +0.063 |
  | mid 2 → 4 | 2.04, −0.88 | −0.051, +0.052 |
  | node 4 | 2.32, 0.00 | −0.061, 0.000 |
  | mid 4 → 14 | 2.29, 1.43 | −0.036, 0.000 |

- **Blending:** rows are weighted by `exp(−d² / 0.35²)`, where `d` is the distance from the row, plus a small "no correction" weight (0.05). Between rows the correction blends smoothly, and about 1 m from every row it fades to zero.
- **Effect along the route:** 2 → 4 gets on average 6.1 cm of correction (max 6.9), 0 → 2 3.4 cm, 4 → 14 2.4 cm (fading out by y ≈ 2), and 12 → 0 1.3 cm (near node 0). **14 → 16, 16 → 18 and 18 → 12 get 0.0 cm**, so the part that already stays in lane is unchanged.
- **Check on the survey logs:**
  - Round 9: mean sideways error 5.5 → 1.0 cm, max 10.8 → 3.9 cm. This is the same data the table was built from, so it only confirms the code does what was intended. The remaining ±2–3 cm is the wander at rest.
  - Round 7 (upper part): unchanged, 2.8 cm.
- **Recalibrating invalidates the table.** It has to be re-measured with a new lower-part survey, the same way as round 9. `check_pose.py` still shows raw readings (no correction).

### Drive 4 result and change to the 0 → 2 rows

- **What the car did (user):** 2 → 4 crossed less than before, but the car was still close to the divider. It **crossed a bit on the 0 → 2 curve**.
- **Drive 4 log** ([logs/vehicle_control.log](../logs/vehicle_control.log), run with the round 9 rows): the route started at once. **0 GPS readings rejected.** `cross_track` stayed between −0.039 and +0.047 m, so the controller held the corrected estimate on the path. The log ends on 4 → 14 at y ≈ 2.3. It can't show where the real car was in the lane, because the correction moves the estimate and the real car together.
- **Why 0 → 2 got worse:** round 9 measured node 0 and mid 0 → 2 reading *toward* the divider (+5.8 and +1.8 cm), so their rows shifted the real car up to 6 cm **toward** the divider. In drive 3, with no correction, the car was already close to the divider there. So the static readings at those two spots don't match how the car behaves when driving.
- **Change (user's suggestion):** the node 0 and mid 0 → 2 rows now use the 2 → 4 average, 6.1 cm **away from** the divider: node 0 `(+0.061, 0.000)`, mid 0 → 2 `(+0.043, +0.043)`. The round 9 values are kept as comments.

Shift of the real car, away from the divider, after the change:

| Segment | Mean | Range |
|---|---|---|
| 0 → 2 | +5.4 cm | +4.4 … +5.9 |
| 2 → 4 | +6.0 cm | +5.3 … +6.9 |
| 4 → 14 | +2.4 cm | 0 … +5.8 |
| 14 → 16, 16 → 18, 18 → 12 | 0 | 0 |
| 12 → 0 | +1.3 cm | 0 … +5.7 (last ~0.7 m before node 0) |

Watch the **outer edge** on 0 → 2 and near node 0: with only ~4 cm clearance per side, a correction larger than the real error moves the car toward the outer line.

### Drive 5 result and second tuning

- **Log:** full lap of the new route and into the second 0 → 2. **0 GPS readings rejected.** `cross_track` within −0.024 … +0.053 m.
- **What the car did (user):** on 0 → 2 the car now ran toward the **outer edge**, with part of it on the outer line, until it reached node 2, where it kept to its lane. On 2 → 4 the **rear left tyre ran on the divider**.

| Region | Correction tried (away from divider) | Result |
|---|---|---|
| 0 → 2 | 0 (drive 3) | close to the divider |
| | about −6 at node 0, −2 mid (round 9 rows, drive 4) | crossed the divider a bit |
| | +6.1 (drive 5) | partly on the outer edge |
| 2 → 4 (mid row) | 0 (drive 3) | crossed visibly |
| | +7.3 (drives 4 and 5) | rear left tyre on the divider |

**Change:** node 0 → `(+0.030, 0)`, mid 0 → 2 → `(+0.021, +0.021)` (3 cm, between the drive 3 and drive 5 results). Mid 2 → 4 → `(−0.064, +0.064)` (9 cm, was 7.3).

Shift of the real car away from the divider now:

| Segment | Mean | Range |
|---|---|---|
| 0 → 2 | +3.2 cm | +2.5 … +5.9 (the top of the range is near node 2) |
| 2 → 4 | +6.7 cm | +5.6 … +8.6 (middle of the curve +8.2) |
| 4 → 14 | +2.4 cm | 0 … +5.8 |
| 14 → 16, 16 → 18, 18 → 12 | 0 | 0 |
| 12 → 0 | +0.7 cm | 0 … +2.8 |

### Drive 6 result

- **Log:** full lap of the new route (passes node 18 and node 12) and into the second lap. **0 GPS readings rejected, no stop.** `cross_track` −0.029 … +0.054 m, mean |error| 1.4 cm.
- **User:** good enough for now. Further 1–2 cm adjustments per drive are possible later.

## 14. Final settings (car A, new route)

All in [qcar/vehicle_control.py](../qcar/vehicle_control.py).

| Setting | Value | Notes |
|---|---|---|
| `nodeSequence` | `[0, 2, 4, 14, 16, 18, 12, 0]` | old loop `[0, 2, 4, 6, 0]` kept as a comment |
| `v_ref` | `0.2` m/s | 0.4 not yet tested on this route |
| `K_stanley` | `1.5` | unchanged |
| `steering_trim` | `-0.14` rad | unchanged |
| Tracked point | `0.1` m ahead | unchanged |
| `gps_scale` | `1.0` | was 1.035 (old calibration) |
| `gps_corrections` | 6 rows, `gps_correction_radius = 0.35` m | §13. Node 0, mid 0 → 2 and mid 2 → 4 tuned from driving |
| `gps_max_jump` / `gps_max_heading_jump` / `gps_timeout` | 0.3 m / 30° / 2 s | §9 |
| `remove_backward_points()` | on route and approach path | §11 |
| Start placement | ~50 cm before the node 0 marker; first `GPS:` line y ≈ 0.3–0.5 | §11, drive 1 |

**Possible next steps:**
- Try `v_ref = 0.4`.
- Fine-tune the correction rows in 1–2 cm steps.
- Walls near the lower part of the map, to reduce the ±2–3 cm wander. This needs a recalibration, a full re-survey and a new correction table.
- The remaining items from [car_A_single_vehicle_setup.md](car_A_single_vehicle_setup.md) §7: per-car settings in `config.txt`, then camera lane correction (see §15) and multi-vehicle.

## 15. Next step: camera lane correction (the better fix for the lower part)

The correction table (§13) treats the symptom: it adds back an error measured once in the lidar readings. It is only as good as those measurements and is invalid after any room or calibration change. Walls near the lower part (§14) would make the lidar position more accurate, but have the same dependence on the room. Even at its best, the lidar position (±2–3 cm) uses most of the ~4 cm clearance.

**Camera lane correction measures the thing that matters:** the car's sideways position between the painted lines, independent of the lidar localization.

| | Correction table (now) | Walls + recalibrate | Camera correction |
|---|---|---|---|
| What it measures | Nothing new: adds back an error measured once | Better lidar position | The car's actual place in the lane |
| Survives room or calibration changes | No | No | Yes |
| Works on car B without retuning | No | No | Mostly (same camera mounting) |
| Effort | Done | Moderate | Most |

**It does not replace the GPS.** The lidar position is still needed to know where the car is on the map, when to turn and which route to follow. The camera only corrects the sideways position within the lane. **Keep the correction table until the camera is proven:** at intersections, or wherever the lines aren't visible, the car falls back to GPS steering, and the table keeps that fallback accurate on the lower part.

**How it would fit the current code:**
- **Where it runs:** [qcar/yolo_server.py](../qcar/yolo_server.py) already reads every RealSense frame. It can also detect the lane lines and send the sideways offset to `vehicle_control.py`, the same way it sends detections now, so there is no second process competing for the camera.
- **Detection:** start simple. Find the yellow divider by colour, then convert the image to a top-down view to get the offset in cm (`LaneKeeping` in `hal.content.qcar_functions` already has that camera geometry). [skills_activities/06-lane_keeping/](../skills_activities/06-lane_keeping/) shows the steps, but it is written for QLabs with keyboard driving and needs adapting.
- **Controller:** when the line is detected confidently, the camera offset replaces the GPS cross-track error in the Stanley term. Otherwise the controller falls back to GPS (with the correction table) as now.

**Plan (measure first, then steer):**
1. **Measure only:** add yellow-line detection to the YOLO server and log the measured offset during a normal drive. The car's behaviour doesn't change.
2. **Check it:** compare the logged offsets with what the car visibly does, especially on the 0 → 2 and 2 → 4 curves.
3. **Use it in the steering,** with GPS as the fallback, once the measurement holds up. Then decide whether the correction table is still needed.
