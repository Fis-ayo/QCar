# tools/

Offline tools: run on the PC (except `on_car/`). The car code lives in [../qcar/](../qcar/), and
`python/start.py` copies **only** `qcar/*.py` to the car, so new tools go here, not in `qcar/`.

| Folder | What goes in it | Files |
|---|---|---|
| [replay/](replay/) | Replaying logged drives through the estimator / steering | `replay_check.py` (run before every estimator change), `c3_replay.py`, `fusion_replay.py`, `fusion_drive_checks.py`, `c3_steer_sim.py` |
| [report/](report/) | Reports on a finished drive | `drive_report.py` (run on every drive) |
| [checks/](checks/) | One-off measurements on logged drives | `camera_drift_check`, `curvature_check`, `curve_shift_check`, `scale_check`, `shift_meas`, `steer_check` |
| [calibration/](calibration/) | Camera calibration solving | `calib_solve.py` |
| [on_car/](on_car/) | Standalone scripts copied to the car by hand (`scp`) and run there | `calib_capture`, `csi_snapshot`, `odometry_log`, `check_pose` |
| [archive/](archive/) | Superseded work (C2 era), kept so the history reports can be reproduced | `c2_offline_test`, `curve_offline_test`, `curve_start_test`, `road_check` |

**Shared setup:** [common.py](common.py). Every PC tool starts with

```python
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
import common
```

which puts `qcar/`, every `tools/` subfolder and `../libraries/python` on the import path, and
gives `common.LOGS`, `common.QCAR`, `common.tool(folder, name)`, `load_vc()` and `pop_opt()`.

Run tools from the project folder, e.g. `python tools/replay/replay_check.py`.
