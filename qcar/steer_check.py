"""
steer_check.py  (PC only)

Per-drive steering check (camera report §47, §50): commanded vs achieved steering per
stretch, achieved = atan(L * gyro / v) with 0.2 s smoothing and the command taken 0.1 s
earlier (servo); time at the steering limit; roundabout in 0.25 m bins; node 2 re-acquisition.

    python qcar/steer_check.py <folder with odometry_drive.csv>
"""
import sys, os
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, '..', '..', 'libraries', 'python'))
from c3_replay import load_vc

D = sys.argv[1]
vc = load_vc()
L = vc['cs_wheelbase']
from hal.products.mats import SDCSRoadMap
route = vc['RoutePath'](vc['remove_backward_points'](
    SDCSRoadMap(leftHandTraffic=False).generate_path(vc['nodeSequence'])))
RL = route.L
o = np.genfromtxt(os.path.join(D, 'odometry_drive.csv'), delimiter=',', names=True, dtype=None, encoding=None)
t = o['t_epoch']; gz = o['gyro_z']; v = o['tach_mps']; cmd = o['delta']
s = np.mod(o['cs_s'], RL); d = o['cs_d_cm']; h = o['cs_h_deg']; src = o['cs_src']
dt = np.median(np.diff(t)); n = max(1, int(round(0.2 / dt)))
ker = np.ones(n) / n
gzs = np.convolve(gz, ker, 'same'); vs = np.convolve(v, ker, 'same')
ach = np.degrees(np.arctan2(L * gzs, np.maximum(vs, 1e-3)))
lag = int(round(0.1 / dt))
cmd_l = np.degrees(np.r_[np.full(lag, np.nan), cmd[:-lag]])
k = route.curvature(s)
moving = vs > 0.25
print(f'route L = {RL:.2f} m, wheelbase {L}, dt {dt*1000:.0f} ms, rows {len(t)}, moving {moving.sum()}')
print(f'drive: s from {o["cs_s"][0]:.2f} to {o["cs_s"][-1]:.2f}  ({o["cs_s"][-1]-o["cs_s"][0]:.2f} m), '
      f'mean speed moving {vs[moving].mean():.3f} m/s')

stretches = [('approach (s>13.5)', (s > 13.5) | (s < 0.0)),
             ('node 0 (0-0.22)', (s >= 0) & (s < 0.22)),
             ('0->2 arc (|k|>0.9, s<2.1)', (s < 2.1) & (abs(k) > 0.9)),
             ('2->4 (2.1-4.0)', (s >= 2.1) & (s < 4.0)),
             ('4->14 straight (|k|<0.05, 4-7.5)', (s >= 4.0) & (s < 7.5) & (abs(k) < 0.05)),
             ('roundabout (|k|>1.2, 7.4-10)', (s >= 7.4) & (s < 10.0) & (abs(k) > 1.2))]
print(f'\n{"stretch":36s} {"n":>4s} {"cmd":>6s} {"ach":>6s} {"need":>6s} {"d_cm":>6s} {"h_deg":>6s}  {"at30%":>5s} src')
for name, m in stretches:
    m = m & moving & ~np.isnan(cmd_l)
    if m.sum() == 0:
        print(f'{name:36s}    0'); continue
    kk = np.median(k[m])
    R = 1 / abs(kk) if abs(kk) > 1e-3 else np.inf
    need = np.sign(kk) * np.degrees(np.arctan(L / np.sqrt(R**2 - L**2))) if np.isfinite(R) else 0.0
    at30 = np.mean(abs(cmd_l[m]) > 29.5) * 100
    srcs = {u: int((src[m] == u).sum()) for u in np.unique(src[m])}
    print(f'{name:36s} {m.sum():4d} {np.mean(cmd_l[m]):+6.1f} {np.mean(ach[m]):+6.1f} {need:+6.1f} '
          f'{np.mean(d[m]):+6.1f} {np.mean(h[m]):+6.1f}  {at30:5.0f} {srcs}')

# longest run at the steering limit
lim = abs(np.degrees(cmd)) > 29.5
best, cur, start = 0, 0, 0
for i, x in enumerate(lim):
    cur = cur + 1 if x else 0
    if cur > best: best, start = cur, i - cur + 1
print(f'\nlongest stretch at |cmd| > 29.5 deg: {best*dt:.2f} s' + (f' at s = {s[start]:.2f}' if best else ''))
# commands 20-30 deg: commanded vs achieved (trim check in curves)
for lo, hi in [(15, 20), (20, 25), (25, 31)]:
    m = moving & (abs(cmd_l) >= lo) & (abs(cmd_l) < hi)
    if m.sum():
        print(f'|cmd| {lo}-{hi} deg: n {m.sum()}, cmd {np.mean(abs(cmd_l[m])):.1f}, ach {np.mean(abs(ach[m])):.1f}')
# roundabout in 0.25 m bins
print('\nroundabout / end, 0.25 m bins: s, cmd, ach, cs_d, cs_h, src')
for a in np.arange(7.25, 10.25, 0.25):
    m = moving & (s >= a) & (s < a + 0.25) & ~np.isnan(cmd_l)
    if m.sum():
        u, c = np.unique(src[m], return_counts=True)
        print(f'  {a:5.2f}  {np.mean(cmd_l[m]):+6.1f} {np.mean(ach[m]):+6.1f}  {np.mean(d[m]):+5.1f} {np.mean(h[m]):+6.1f}  {u[np.argmax(c)]}')
# node 2 re-acquisition: first non-DR camera fit after the 0->2 gap
print('\n0->2 gap and node 2: s, cs_d_cm, src (src changes)')
prev = None
for i in range(len(s)):
    if 0.2 < s[i] < 3.0 and src[i] != prev:
        print(f'  s={s[i]:.2f}  d={d[i]:+.1f}  h={h[i]:+.1f}  src={src[i]}')
        prev = src[i]
