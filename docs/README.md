# Docs

| Folder / file | What it is | Kept up to date? |
|---|---|---|
| [car_A_setup.md](car_A_setup.md) | **Start here.** Car A as it is now: settings, room requirements, how to run and check it, limitations, next steps | Yes: update it whenever a setting or procedure changes |
| [car_B_setup.md](car_B_setup.md) | Car B as it is now. Runs car A's code; points to car A's doc for anything shared | Yes: update it whenever a setting or procedure changes |
| [plans/](plans/) | Work that is planned but not started | Updated when that work starts or changes |
| [history/](history/) | Finished, dated records of how problems were found and fixed | No: written once, kept as written |

**Plans**
- [plans/camera_lane_correction.md](plans/camera_lane_correction.md): measure the car's position in the lane with the camera, to replace the GPS correction table.
- [plans/supervisor_feedback_2026-11-05.md](plans/supervisor_feedback_2026-11-05.md): Dr. Fu's feedback on the 2026-10-30 update, the decisions taken, the controlled-comparison plan and the order of work after the camera steps (then multi-vehicle).

**History** (oldest first)
- [history/2026-10-26_old_loop_tuning.md](history/2026-10-26_old_loop_tuning.md): car A on the old loop `0 → 2 → 4 → 6 → 0`.
- [history/2026-10-28_new_route_investigation.md](history/2026-10-28_new_route_investigation.md): making the route `0 → 2 → 4 → 14 → 16 → 18 → 12 → 0` work.
- [history/2026-10-29_car_B_first_runs.md](history/2026-10-29_car_B_first_runs.md): car A's code on car B (library fix, first full lap). Car B's first camera-steering drive (2026-11-20, no car-specific tuning needed) is §61 of the camera lane correction report.

**Conventions**
- **A new investigation** gets a new dated file in `history/`. When it is finished, summarise the outcome and update the settings in the setup doc.
- **Each car** has its own `car_<X>_setup.md`. Things shared by all cars (room layout, survey procedure) can later move into a shared doc; for now `car_B_setup.md` links to `car_A_setup.md` for them.
