# Docs

| Folder / file | What it is | Kept up to date? |
|---|---|---|
| [car_A_setup.md](car_A_setup.md) | **Start here.** Car A as it is now: settings, room requirements, how to run and check it, limitations, next steps | Yes: update it whenever a setting or procedure changes |
| [plans/](plans/) | Work that is planned but not started | Updated when that work starts or changes |
| [history/](history/) | Finished, dated records of how problems were found and fixed | No: written once, kept as written |

**Plans**
- [plans/camera_lane_correction.md](plans/camera_lane_correction.md): measure the car's position in the lane with the camera, to replace the GPS correction table.

**History** (oldest first)
- [history/2026-10-26_old_loop_tuning.md](history/2026-10-26_old_loop_tuning.md): car A on the old loop `0 → 2 → 4 → 6 → 0`.
- [history/2026-10-28_new_route_investigation.md](history/2026-10-28_new_route_investigation.md): making the route `0 → 2 → 4 → 14 → 16 → 18 → 12 → 0` work.

**Conventions**
- **A new investigation** gets a new dated file in `history/`. When it is finished, summarise the outcome and update the settings in the setup doc.
- **When car B is added,** it gets its own `car_B_setup.md`. Things shared by all cars (room layout, survey procedure) can then move into a shared doc.
