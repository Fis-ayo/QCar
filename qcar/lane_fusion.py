"""
lane_fusion.py

Camera report "Where to pick up" step 3 (2026-11-19): confidence-weighted fusion of
encoder + gyro (prediction), front-camera lane fits and GPS (updates), on the map.
Pure numpy so vehicle_control.py can import it later; for now only fusion_replay.py
uses it (offline).

State x = [s, d, psi, b_al, b_lat]
    s     position of the front axle along the route (m, unwrapped)
    d     sideways offset of the front axle from the lane centre (m, + = left)
    psi   car body heading in map coordinates (rad)
    b_al, b_lat  GPS position error along / across the route (m). The raw GPS is
          biased by place (+6 cm at node 0, -8 cm on 2 -> 4 in the 2026-11-19 drive,
          the error the old correction table held), constant over ~1 m: modelled as
          a first-order Gauss-Markov process in distance (sd gps_sd_bias, correlation
          length gps_bias_len). The camera makes it observable; where the camera is
          blind the GPS still bounds the drift, as far as the bias allows.
Derived: heading to the lane h = psi - th(s), th = route tangent at the front axle;
expected h when following the lane h_exp(s) = th(s - L) - th(s) (rear axle on the lane;
used only for the camera heading checks).
Prediction (§53): the rear axle is carried along the body heading, the front axle is
rear + L (axle_step); before §53 the offset was moved as the rear axle's ('old').

Why psi and not h as the state: with h = psi - th(s) the filter knows that an error
in s becomes a heading error kappa * ds on an arc (report §46, the roundabout), so the
offset uncertainty grows fast on tight curves when s is uncertain, and slowly on
straights.

Updates (each with an honest uncertainty; one chi-square innovation test decides
whether a reading is used, instead of the jump gate / confirmation / re-lock rules):
    camera: d (always) and h (only where the C3 rules allow a camera heading)
    GPS:    'full'  -> x, y and heading of the GPS point (gps_ahead behind the front
                       axle), with the bias states
            'white' -> same, but no bias states: every reading independent (shows why
                       the bias is needed)
            'along' -> only the position along the route (as C3 does now)
            'none'  -> not used
"""
import numpy as np


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


ENC_AXLES = ('front', 'rear', 'mean')
LANE_HALF = 0.1285   # m, lane centre to its boundary (lane width 25.7 cm)


def axle_step(route, L, s, d, psi, ds_enc, dpsi, enc_axle='front'):
    """Dead reckoning of the FRONT-axle offset, done through the rear axle (report §53).

    The rear axle moves exactly along the body heading (no side slip), so it is
    carried by the encoder and gyro; the front axle is rear + L along the body.
    The old model (d += sin(psi - th(s - L)) * ds) moved the rear axle's offset but
    the camera and Stanley read d as the front axle's: on arcs the car steered the
    rear axle onto the lane and the front ran 3.8 cm (0 -> 2) / 5.0 cm (roundabout)
    outward.
    The steering angle comes from the gyro and the encoder, sin(delta) = L * dpsi / ds_front.
    enc_axle: which axle's travel the scaled encoder distance is ('front', 'rear', or
    'mean' of both, e.g. one motor through a centre differential).
    s, d: front axle along / across the route (m); psi: body heading (rad).
    -> (s, d, psi) after the step.
    """
    psi2 = psi + dpsi
    if abs(ds_enc) < 1e-9:
        return s, d, psi2
    # front / rear axle travel from the encoder distance and the turn
    ds_f = ds_enc
    for _ in range(3 if enc_axle == 'mean' else 1):
        sin_d = np.clip(L * dpsi / ds_f, -0.95, 0.95)
        cos_d = np.sqrt(1 - sin_d ** 2)
        if enc_axle == 'front':
            break
        if enc_axle == 'rear':
            ds_f = ds_enc / np.cos(np.arctan(L * dpsi / ds_enc))
            cos_d = ds_enc / ds_f
            break
        ds_f = 2 * ds_enc / (1 + cos_d)       # mean: ds_enc = (ds_f + ds_f cos_d) / 2
    ds_r = ds_f * cos_d
    th = float(route.heading(s))
    F = route.xy(s) + d * np.array([-np.sin(th), np.cos(th)])
    R = F - L * np.array([np.cos(psi), np.sin(psi)])
    pm = psi + 0.5 * dpsi
    R = R + ds_r * np.array([np.cos(pm), np.sin(pm)])
    F = R + L * np.array([np.cos(psi2), np.sin(psi2)])
    # front axle back onto the route: along-route position and offset
    s2 = s + ds_f
    for _ in range(2):
        th2 = float(route.heading(s2))
        v = F - route.xy(s2)
        s2 += v[0] * np.cos(th2) + v[1] * np.sin(th2)
    th2 = float(route.heading(s2))
    v = F - route.xy(s2)
    return s2, float(-v[0] * np.sin(th2) + v[1] * np.cos(th2)), psi2


DEFAULTS = dict(
    # process noise
    q_s=0.05,              # m per sqrt(m): along-route random walk (encoder scale/slip; §46: ~17 cm after 5.6 m)
    q_d=0.005,             # m per sqrt(m): sideways slip / model error
    q_psi=np.radians(0.5), # rad per sqrt(m): gyro noise/bias drift
    q_psi_scale=0.02,      # rad per sqrt(rad turned): gyro scale error
    # camera fit uncertainty: sd = sqrt(sd0[src]^2 + (k_rms * rms)^2)
    cam_sd0={'YE': 0.010, 'E': 0.015, 'Y': 0.020},
    cam_k_rms=1.0,
    # camera heading sd: 6 deg since §64 (was 2/3/3): on arcs the camera heading reads ~2 deg
    # outward of the model and up to 8 deg in drive F's roundabout; 3 deg made the filter reject
    # a good fit there. 6 deg: no rejection on 9 drives, re-acquisition error 3.8 -> 3.6 cm.
    cam_sd_h={'YE': np.radians(4.0), 'E': np.radians(6.0), 'Y': np.radians(6.0)},
    # GPS uncertainty (route frame)
    gps_sd_along=0.03,     # 'white' / 'along' modes: total GPS error
    gps_sd_lat=0.04,
    gps_sd_head=np.radians(5.0),
    gps_sd_noise=0.015,    # 'full' mode: reading-to-reading noise ...
    gps_sd_bias=0.06,      # ... plus the slowly varying bias (sd) ...
    gps_bias_len=1.0,      # ... correlated over this distance (m)
    # innovation tests (chi-square), ~99.7 %
    chi2={1: 9.0, 2: 11.8, 3: 14.2},
    # initial uncertainty
    p0_s=0.05, p0_d=0.05, p0_psi=np.radians(5.0),
    # dead-reckoning model (§53): 'axle' or 'old'; which axle the encoder distance is
    dr_model='old',       # 'axle' not validated yet (§54)
    enc_axle='front',
)


class LaneFusion:
    def __init__(self, route, wheelbase, gps_ahead, s0, psi0, d0=0.0, **kw):
        self.r = route
        self.L = wheelbase
        self.a = gps_ahead
        self.p = dict(DEFAULTS)
        self.p.update(kw)
        self.x = np.array([s0, d0, psi0, 0.0, 0.0], float)
        sb = self.p['gps_sd_bias']
        self.P = np.diag([self.p['p0_s'], self.p['p0_d'], self.p['p0_psi'], sb, sb]) ** 2

    # --- geometry ----------------------------------------------------------
    def th(self, s):
        return float(self.r.heading(s))

    def k(self, s):
        return float(self.r.curvature(s))

    def h(self):
        return float(wrap(self.x[2] - self.th(self.x[0])))

    def h_exp(self, s=None):
        s = self.x[0] if s is None else s
        return float(wrap(self.th(s - self.L) - self.th(s)))

    def sd(self):
        return np.sqrt(np.diag(self.P))

    # --- prediction (every odometry sample) ----------------------------------
    def predict(self, ds, dpsi):
        """ds: m from the encoder (x scale), dpsi: gyro turn (rad).

        p['dr_model']: 'old' (default, the car's model: it carries the rear axle's offset
        as the front's) or 'axle' (§53, offline test only, not validated, §54: rear axle
        carried, front by geometry; p['enc_axle'] says which axle the encoder distance is)."""
        s, d, psi, bal, blat = self.x
        p = self.p
        phi = np.exp(-abs(ds) / p['gps_bias_len'])
        F = np.eye(5)
        F[3, 3] = F[4, 4] = phi
        if p['dr_model'] == 'old':
            e = wrap(psi - self.th(s) - self.h_exp(s))      # heading relative to following the lane
            F[1, 0] = np.cos(e) * ds * (-self.k(s - self.L))  # d(e)/ds = -kappa(s - L)
            F[1, 2] = np.cos(e) * ds
            s2, d2 = s + ds, d + np.sin(e) * ds
        else:
            s2, d2, _ = axle_step(self.r, self.L, s, d, psi, ds, dpsi, p['enc_axle'])
            for j, eps in ((0, 1e-4), (1, 1e-4), (2, 1e-5)):   # numerical Jacobian of (s, d)
                x = [s, d, psi]
                x[j] += eps
                sj, dj, _ = axle_step(self.r, self.L, x[0], x[1], x[2], ds, dpsi, p['enc_axle'])
                F[0, j], F[1, j] = (sj - s2) / eps, (dj - d2) / eps
        self.x = np.array([s2, d2, psi + dpsi, phi * bal, phi * blat])
        qb = p['gps_sd_bias'] ** 2 * (1 - phi ** 2)
        Q = np.diag([p['q_s'] ** 2 * abs(ds),
                     p['q_d'] ** 2 * abs(ds),
                     p['q_psi'] ** 2 * abs(ds) + p['q_psi_scale'] ** 2 * abs(dpsi),
                     qb, qb])
        self.P = F @ self.P @ F.T + Q

    # --- generic update -----------------------------------------------------
    def _update(self, y, H, R, force=False):
        """y: innovation, H: Jacobian, R: noise. -> (accepted, nis)."""
        S = H @ self.P @ H.T + R
        Si = np.linalg.inv(S)
        nis = float(y @ Si @ y)
        if not force and nis > self.p['chi2'][len(y)]:
            return False, nis
        K = self.P @ H.T @ Si
        self.x = self.x + K @ y
        I_KH = np.eye(5) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ R @ K.T   # Joseph form
        return True, nis

    # --- camera -------------------------------------------------------------
    def camera(self, src, d_m, h_rad=None, rms_m=0.0, force=False):
        """Lane fit from the front camera. h_rad=None: offset only."""
        p = self.p
        sd_d = np.hypot(p['cam_sd0'][src], p['cam_k_rms'] * rms_m)
        if h_rad is None:
            y = np.array([d_m - self.x[1]])
            H = np.array([[0.0, 1.0, 0.0, 0.0, 0.0]])
            R = np.array([[sd_d ** 2]])
        else:
            y = np.array([d_m - self.x[1], wrap(h_rad - self.h())])
            H = np.array([[0.0, 1.0, 0.0, 0.0, 0.0], [-self.k(self.x[0]), 0.0, 1.0, 0.0, 0.0]])
            R = np.diag([sd_d ** 2, p['cam_sd_h'][src] ** 2])
        return self._update(y, H, R, force)

    def camera_s(self, s_true, sd_m, force=False):
        """Along-route position of the front axle measured by the camera where the lane
        curvature changes in view (lane_camera.shift_fit, report §55-§61): s_true =
        the s the frame's profile was made at + the best shift."""
        y = np.array([s_true - self.x[0]])
        H = np.array([[1.0, 0.0, 0.0, 0.0, 0.0]])
        return self._update(y, H, np.array([[sd_m ** 2]]), force)

    # --- GPS ----------------------------------------------------------------
    def _gps_point(self, x, bias=True):
        s, d, psi, bal, blat = x
        th = self.th(s)
        fx, fy = self.r.xy(s)
        fx, fy = fx - d * np.sin(th), fy + d * np.cos(th)
        gx, gy = fx - self.a * np.cos(psi), fy - self.a * np.sin(psi)
        if bias:
            tg = self.th(s - self.a)
            gx += bal * np.cos(tg) - blat * np.sin(tg)
            gy += bal * np.sin(tg) + blat * np.cos(tg)
        return np.array([gx, gy])

    def gps(self, gx, gy, gth, mode='full'):
        p = self.p
        th = self.th(self.x[0] - self.a)
        if mode == 'along':
            sG, _ = self.r.project(gx, gy, self.x[0] - self.a, 0.6)
            y = np.array([sG + self.a - self.x[0]])
            H = np.array([[1.0, 0.0, 0.0, 0.0, 0.0]])
            return self._update(y, H, np.array([[p['gps_sd_along'] ** 2]]))
        # full / white: x, y, heading
        bias = mode == 'full'
        z0 = self._gps_point(self.x, bias)
        H = np.zeros((3, 5))
        for j, eps in enumerate((1e-3, 1e-4, 1e-4, 1e-4, 1e-4)):
            if j >= 3 and not bias:
                continue
            dx = np.zeros(5)
            dx[j] = eps
            H[:2, j] = (self._gps_point(self.x + dx, bias) - z0) / eps
        H[2, 2] = 1.0
        y = np.array([gx - z0[0], gy - z0[1], wrap(gth - self.x[2])])
        Rot = np.array([[np.cos(th), -np.sin(th)], [np.sin(th), np.cos(th)]])
        sa, sl = ((p['gps_sd_noise'], p['gps_sd_noise']) if bias else (p['gps_sd_along'], p['gps_sd_lat']))
        R = np.zeros((3, 3))
        R[:2, :2] = Rot @ np.diag([sa ** 2, sl ** 2]) @ Rot.T
        R[2, 2] = p['gps_sd_head'] ** 2
        return self._update(y, H, R)
