"""
c3_steer_sim.py  (PC only)

Camera report §44-§46: kinematic simulation of the camera-steering loop (Stanley
from the front axle, heading error from the exact lane tangent). SteeringController,
RoutePath, remove_backward_points and the settings are taken from
vehicle_control.py's source; the steering mirrors its camera-steering block.
(The curvature feedforward variants tested in §44-§45 were removed with the
feedforward code, §46.)

Model: kinematic bicycle (rear-axle reference, wheelbase cs_wheelbase), constant
speed, optional first-order steering-servo lag. The estimator knows the position
along the route exactly; offset/heading come from camera frames (rate, delay,
noise) and are carried on between frames by ideal dead reckoning. Not modelled:
the blind 0 -> 2 stretch (ideal DR there is the same as camera here), slip,
steering trim errors.

    python qcar/c3_steer_sim.py
"""
import ast
import os
import sys
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', '..', 'libraries', 'python'))


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def load_vc():
    src = open(os.path.join(HERE, 'vehicle_control.py'), encoding='utf-8').read()
    keep = []
    for node in ast.parse(src).body:
        if isinstance(node, (ast.ClassDef, ast.FunctionDef)) and node.name in (
                'RoutePath', 'remove_backward_points', 'SteeringController'):
            keep.append(node)
        elif isinstance(node, ast.Assign) and len(node.targets) == 1 and isinstance(node.targets[0], ast.Name):
            if node.targets[0].id.startswith(('cs_', 'nodeSequence', 'K_stanley', 'K_soft', 'v_ref', 'v_curve', 'use_curve_speed')):
                keep.append(node)
    ns = {'np': np, 'wrap_to_pi': wrap}
    exec(compile(ast.Module(body=keep, type_ignores=[]), 'vehicle_control.py', 'exec'), ns)
    return ns


# segments of the route by s (m), from vehicle_control's node positions
SEGS = [('0->2', 0.0, 1.95), ('2->4', 1.95, 3.86), ('4->14', 3.86, 6.74), ('14->16', 6.74, 9.83),
        ('16->18', 9.83, 10.56), ('18->12', 10.56, 12.01), ('12->0', 12.01, 99.0)]


def simulate(vc, route, wp, v, delay, rate=13.0, noise_d=0.005, noise_h=np.radians(1),
             servo_tau=0.0, d0=0.02, s_start=None, dist=None, dt=0.005, seed=0,
             k_soft=0.0, curve_speed=False, speed_tau=0.3):
    rng = np.random.default_rng(seed)
    L = vc['cs_wheelbase']
    s_start = route.L - 0.30 if s_start is None else s_start
    dist = route.L + 0.5 if dist is None else dist
    ctrl = vc['SteeringController'](waypoints=wp, k=vc['K_stanley'], k_soft=k_soft)
    v_ref, vel = v, v
    wpS = np.r_[0, np.cumsum(np.hypot(*np.diff(wp, axis=1)))]
    ctrl.wpi = int(np.clip(np.searchsorted(wpS, np.mod(s_start, route.L), 'right') - 1, 0, ctrl.N - 2))
    # start: front axle on the route at s_start, d0 to the left, heading = lane tangent
    th = float(route.heading(s_start))
    nL = np.array([-np.sin(th), np.cos(th)])
    front = route.xy(s_start) + nL * d0
    rear = front - L * np.array([np.cos(th), np.sin(th)])
    s = s_start
    delta = 0.0
    hist = []                       # (t, d_true, h_true)
    err_d = err_h = 0.0
    next_frame = 0.0
    t = 0.0
    out = []
    while s - s_start < dist:
        front = rear + L * np.array([np.cos(th), np.sin(th)])
        s, d_true = route.project(front[0], front[1], s, 0.05)
        h_true = float(wrap(th - route.heading(s)))
        hist.append((t, d_true, h_true))
        if t >= next_frame:
            next_frame += 1.0 / rate
            # measurement of the state `delay` s ago, treated as current
            j = max(0, len(hist) - 1 - int(round(delay / dt)))
            err_d = hist[j][1] + rng.normal(0, noise_d) - d_true
            err_h = hist[j][2] + rng.normal(0, noise_h) - h_true
        d_est, h_est = d_true + err_d, h_true + err_h      # ideal DR between frames
        thPath = float(route.heading(s))
        nL = np.array([-np.sin(thPath), np.cos(thPath)])
        p_steer = route.xy(s) + nL * d_est
        vt = v_ref
        if curve_speed:          # as vehicle_control's curvature speed limit
            kMax = float(np.max(np.abs(route.curvature(s + np.linspace(-L, vc['v_curve_ahead'], 40)))))
            if kMax > 1e-6:
                vt = min(v_ref, max(vc['v_curve_min'], float(np.sqrt(vc['v_curve_alat'] / kMax))))
        vel += (vt - vel) * (1 - np.exp(-dt / speed_tau))   # speed loop as a first-order lag
        v = vel
        cmd = ctrl.update(p_steer, thPath + h_est, v, thPath)   # as vehicle_control's camera-steering block
        delta = cmd if servo_tau <= 0 else delta + (cmd - delta) * (1 - np.exp(-dt / servo_tau))
        rear = rear + v * dt * np.array([np.cos(th), np.sin(th)])
        th = th + v * dt * np.tan(delta) / L
        t += dt
        out.append((s, d_true, delta, v, t))
    return np.array(out)


def seg_max(res, route):
    sm = np.mod(res[:, 0], route.L)
    r = {}
    for name, a, b in SEGS:
        m = (sm >= a) & (sm < b)
        r[name] = 100 * np.max(np.abs(res[m, 1])) if np.any(m) else np.nan
    return r


def main():
    vc = load_vc()
    from hal.products.mats import SDCSRoadMap
    rm = SDCSRoadMap(leftHandTraffic=False)
    wp = vc['remove_backward_points'](rm.generate_path(vc['nodeSequence']))
    route = vc['RoutePath'](wp)
    print(f"route length {route.L:.2f} m, K_stanley {vc['K_stanley']}, wheelbase {vc['cs_wheelbase']} m")
    print("max |offset| (cm) per segment, one lap from the straight before node 0, start 2 cm off\n")
    hdr = f"{'case':44s}" + ''.join(f"{n:>8s}" for n, _, _ in SEGS)
    print(hdr)
    def seg_steer(res):
        sm = np.mod(res[:, 0], route.L)
        return {n: np.degrees(np.max(np.abs(res[(sm >= a) & (sm < b), 2]))) for n, a, b in SEGS}
    print("(second line of each case: largest |steering| in deg; lap time)")
    for delay, servo in ((0.06, 0.0), (0.12, 0.1)):
        for label, kw in (("0.4 const, K_soft 0 (before)", dict(k_soft=0.0, curve_speed=False)),
                          ("K_soft only", dict(k_soft=vc['K_soft'], curve_speed=False)),
                          ("K_soft + curve speed (now)", dict(k_soft=vc['K_soft'], curve_speed=True))):
            res = simulate(vc, route, wp, vc['v_ref'], delay, servo_tau=servo, **kw)
            r, st = seg_max(res, route), seg_steer(res)
            name = f"{delay*1000:.0f}ms servo {servo*1000:.0f}ms {label}"
            print(f"{name:44s}" + ''.join(f"{r[n]:8.1f}" for n, _, _ in SEGS))
            print(f"{'   steering / lap %.1f s' % res[-1, 4]:44s}" + ''.join(f"{st[n]:8.1f}" for n, _, _ in SEGS))

if __name__ == '__main__':
    main()
