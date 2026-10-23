#!/usr/bin/env python3
"""
Quaternion multicopter + sensors + Kalman filtering + DYNAMIC 3-D obstacle avoidance in PyBullet
(ALL IN ONE FILE).

    pip install pybullet numpy scipy matplotlib
    python quad_pybullet.py                  # PyBullet window; NEW random world every run
    python quad_pybullet.py --seed 5         # repeat the same world
    python quad_pybullet.py --no-gui         # headless, fast, plots saved to results/
    python quad_pybullet.py --compare        # also run once WITHOUT avoidance (same world)
    python quad_pybullet.py --print-every 0.25   # terminal table interval (0 = off)
    python quad_pybullet.py --learn 12 1 2 3     # offline parameter tuning

Every run: random obstacle layout (type order, position, size, speed, direction) + noisy reference.
The terminal prints the drone's coordinates next to the ORIGINAL reference path coordinates.
"""
import os
import sys
import time
import argparse
import tempfile
import numpy as np
import matplotlib
if '--no-gui' in sys.argv or '--no-show' in sys.argv or '--learn' in sys.argv:
    matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.linalg import expm
import pybullet as p
import pybullet_data


# ============================================================================
# 1. CONFIG + RANDOM WORLD
# ============================================================================

class Config:
    def __init__(self):
        self.seed = None                  # None = NEW random scenario every run (printed, repeat with --seed N)
        self.printEvery = 0.5             # [s] terminal table: drone coordinates vs reference (0 = off)
        # ---------------- vehicle (paper / your init_params.m) ----------------
        self.m = 0.025
        self.J = np.array([4.856e-3, 4.856e-3, 8.801e-3])
        self.g = 9.81
        self.CD, self.rho_air, self.A = 0.8, 1.225, 0.01425      # drag, paper eq. (30)
        self.Tmax = 3 * self.m * self.g
        self.droneRadius = 0.20                                   # collision sphere [m]
        # ---------------- timing ----------------------------------------------
        self.dt = 0.005                                           # 200 Hz physics + attitude loop
        self.Ttraj = 55.0
        self.tHold = 4.0
        self.posLoopEvery = 4                                     # 50 Hz
        self.sensorEvery = 10                                     # 20 Hz GPS + lidar
        self.planEvery = 20                                       # 10 Hz planner
        # ---------------- mission + reference ---------------------------------
        self.waypoints = np.array([
            [1.80, 1.95, 2.10, 2.30, 2.50, 2.70, 2.90],
            [2.27, 2.40, 2.55, 2.70, 2.82, 2.92, 3.00],
            [0.60, 0.70, 0.82, 0.95, 1.05, 1.12, 1.20]])
        self.missionScale = 10.0
        self.refSamples = 150
        self.refNoiseStd = 0.04            # random noise added to every raw path point [m]
        self.bsplineDegree = 5
        self.nCtrl = 10
        # ---------------- controller gains (critically damped) ---------------
        self.Kp, self.Kd, self.Ki = 16.0, 8.0, 2.0
        self.Kpq, self.Kdq = 900.0, 60.0
        self.Kiq = np.array([20.0, 20.0, 10.0])
        self.q3r = 0.0
        self.cmdFilterWn = 40.0
        # ---------------- disturbances + sensor noise ------------------------
        self.windStd = 0.004
        self.gust = (14.0, 1.0, 0.02, -0.015, 0.0)               # t0, duration, Fx, Fy, Fz
        self.gpsStd, self.accStd, self.gyroStd, self.attStd = 0.02, 0.05, 0.003, 0.0005
        # ---------------- obstacle sensor: 3-D LiDAR made of PyBullet rays ---
        self.sensorRange = 2.5
        self.sensorStd = 0.02
        self.nRays = 800                                          # rays over the whole sphere
        # ---------------- avoidance planner ----------------------------------
        self.enableAvoidance = True
        self.margin = 0.15
        self.deltaWn = 2.2
        self.planHorizon = 2.5
        self.planStep = 0.2
        self.nDirs, self.nMags = 64, 28
        self.maxOffset = 1.6
        self.wVert, self.wPrev = 1.5, 0.6
        self.slack, self.wSwitch = 0.04, 0.10
        self.zMin, self.zMax = 0.35, 11.0
        # world (buildings / moving spheres) is created in world_layout()

    @property
    def rho(self):                      # required clearance from obstacle surfaces
        return self.droneRadius + self.margin


def world_layout(cfg, ref):
    """RANDOM world (different every run unless --seed is given): buildings and moving spheres are
    placed near the original path with random positions, sizes, heights, speeds and directions.
    Buildings [cx,cy,sx,sy,z0,z1];  moving spheres [t_hit,vx,vy,vz,r,ox,oy,oz]."""
    rng = np.random.default_rng([int(cfg.seed), 1])
    P = ref.waypoints
    seg = np.sqrt((np.diff(P, axis=1) ** 2).sum(0))
    cs = np.r_[0, np.cumsum(seg)]
    cs /= cs[-1]
    pp = lambda f: np.array([np.interp(f, cs, P[i]) for i in range(3)])
    sm = lambda x: 10 * x ** 3 - 15 * x ** 4 + 6 * x ** 5
    pStart, pEnd = P[:, 0], P[:, -1]
    for attempt in range(200):
        roles = ['tower', 'block', 'bridge']
        rng.shuffle(roles)                                           # which kind of building comes first is random
        fr = [rng.uniform(0.07, 0.14), rng.uniform(0.34, 0.46), rng.uniform(0.56, 0.68)]
        buildings = []
        for role, f in zip(roles, fr):
            a = pp(f)
            d = pp(min(f + 0.01, 1)) - pp(max(f - 0.01, 0)); d[2] = 0
            d = d / (np.linalg.norm(d) + 1e-9)
            lat = np.array([-d[1], d[0], 0]) * rng.uniform(-0.25, 0.25)   # random sideways offset from the path
            c = a + lat
            if role == 'tower':
                s_ = rng.uniform(0.5, 0.8);  buildings.append([c[0], c[1], s_, s_, 0.0, 12.0])
            elif role == 'block':
                s_ = rng.uniform(0.8, 1.2);  buildings.append([c[0], c[1], s_, s_, 0.0, a[2] + rng.uniform(0.0, 0.2)])
            else:
                s_ = rng.uniform(1.0, 1.4);  z0 = a[2] - rng.uniform(0.12, 0.25)
                buildings.append([c[0], c[1], s_, s_, z0, z0 + rng.uniform(1.2, 1.8)])
        dyn = []
        for lo, hi in [(0.18, 0.30), (0.74, 0.88)]:
            f = rng.uniform(lo, hi)
            t_hit = cfg.Ttraj * _bisect(sm, f)
            d = pp(min(f + 0.01, 1)) - pp(max(f - 0.01, 0)); d[2] = 0
            d = d / (np.linalg.norm(d) + 1e-9)
            speed = rng.uniform(0.10, 0.25)
            if rng.random() < 0.5:                                    # crossing sideways (random side / slight angle)
                ang = rng.uniform(-0.5, 0.5) + (np.pi / 2) * rng.choice([-1, 1])
                v = speed * np.array([d[0] * np.cos(ang) - d[1] * np.sin(ang), d[0] * np.sin(ang) + d[1] * np.cos(ang), 0])
            else:                                                     # dropping from above / rising from below
                v = np.array([0, 0, speed * rng.choice([-1, 1])])
            dyn.append([t_hit, v[0], v[1], v[2], rng.uniform(0.15, 0.22), *rng.uniform(-0.12, 0.12, 3)])
        dyn = np.array(dyn)
        dyn[:, 1:4] *= getattr(cfg, 'dyn_speed_scale', 1.0)
        # ---- sanity: nothing may sit on the start or the destination ------------------------------------
        ok = True
        for b in buildings:
            for q in (pStart, pEnd):
                if _box_clear(q, b) < cfg.rho + 0.3: ok = False
        for d in dyn:
            hit = ref.eval(d[0])[0] + d[5:8]
            p0 = hit - d[1:4] * d[0]
            for t_chk, q in ((0.0, pStart), (cfg.Ttraj + cfg.tHold, pEnd)):
                if np.linalg.norm(p0 + d[1:4] * t_chk - q) - d[4] < cfg.rho + 0.4: ok = False
        if ok:
            return np.array(buildings), dyn
    raise RuntimeError('could not generate a valid random scenario')


def _box_clear(q, b):
    c = np.array([b[0], b[1], (b[4] + b[5]) / 2]); h = np.array([b[2] / 2, b[3] / 2, (b[5] - b[4]) / 2])
    qq = np.abs(q - c) - h
    return np.linalg.norm(np.maximum(qq, 0)) + min(max(qq), 0)


def _bisect(fun, y):
    lo, hi = 0.0, 1.0
    for _ in range(60):
        mid = (lo + hi) / 2
        if fun(mid) < y:
            lo = mid
        else:
            hi = mid
    return (lo + hi) / 2


# ============================================================================
# 2. B-SPLINE REFERENCE
# ============================================================================

def basis_all(t, p, u):
    """All B-spline basis functions of degree p at u (Piegl & Tiller A2.2, Cox-de Boor)."""
    n = len(t) - p - 1
    N = np.zeros(n)
    k = n - 1 if u >= t[n] else int(np.clip(np.searchsorted(t, u, side='right') - 1, p, n - 1))
    left = np.zeros(p + 1); right = np.zeros(p + 1); Nl = np.zeros(p + 1); Nl[0] = 1.0
    for j in range(1, p + 1):
        left[j] = u - t[k + 1 - j]
        right[j] = t[k + j] - u
        saved = 0.0
        for r in range(j):
            temp = Nl[r] / (right[r + 1] + left[j - r])
            Nl[r] = saved + right[r + 1] * temp
            saved = left[j - r] * temp
        Nl[j] = saved
    N[k - p:k + 1] = Nl
    return N


def deriv_ctrl(C, t, p):
    Cd = np.array([p * (C[i + 1] - C[i]) / (t[i + p + 1] - t[i + 1]) for i in range(len(C) - 1)])
    return Cd, t[1:-1], p - 1


class Reference:
    def __init__(self, cfg, rng):
        P = cfg.waypoints
        P = P[:, :1] + cfg.missionScale * (P - P[:, :1])
        self.waypoints = P
        seg = np.sqrt((np.diff(P, axis=1) ** 2).sum(0))
        cs = np.r_[0, np.cumsum(seg)]
        s = np.linspace(0, cs[-1], cfg.refSamples)
        D = np.stack([np.interp(s, cs, P[i]) for i in range(3)], axis=1)
        noise = cfg.refNoiseStd * rng.standard_normal(D.shape)
        noise[[0, -1]] = 0.0                                  # start / goal exact
        Dn = D + noise                                        # NOISY raw path
        self.raw, self.ideal = Dn, D

        p, n = cfg.bsplineDegree, cfg.nCtrl
        d = np.r_[0, np.cumsum(np.sqrt((np.diff(Dn, axis=0) ** 2).sum(1)))]
        u = d / d[-1]
        nint = n - p - 1
        t = np.r_[np.zeros(p + 1), np.arange(1, nint + 1) / (nint + 1), np.ones(p + 1)]
        Nm = np.array([basis_all(t, p, ui) for ui in u])
        rhs = Dn[1:-1] - np.outer(Nm[1:-1, 0], Dn[0]) - np.outer(Nm[1:-1, -1], Dn[-1])
        Cint = np.linalg.lstsq(Nm[1:-1, 1:-1], rhs, rcond=None)[0]       # least squares
        self.C0 = np.vstack([Dn[0], Cint, Dn[-1]])
        self.t0, self.p0 = t, p
        self.C1, self.t1, self.p1 = deriv_ctrl(self.C0, self.t0, self.p0)
        self.C2, self.t2, self.p2 = deriv_ctrl(self.C1, self.t1, self.p1)
        self.T = cfg.Ttraj

    def curve(self, n=400):
        return np.array([basis_all(self.t0, self.p0, u) @ self.C0 for u in np.linspace(0, 1, n)])

    def eval(self, t):
        T = self.T
        tau = min(max(t / T, 0.0), 1.0)
        u = 10 * tau ** 3 - 15 * tau ** 4 + 6 * tau ** 5               # smootherstep time law
        ud = (30 * tau ** 2 - 60 * tau ** 3 + 30 * tau ** 4) / T
        udd = (60 * tau - 180 * tau ** 2 + 120 * tau ** 3) / T ** 2
        c0 = basis_all(self.t0, self.p0, u) @ self.C0
        c1 = basis_all(self.t1, self.p1, u) @ self.C1
        c2 = basis_all(self.t2, self.p2, u) @ self.C2
        return c0, c1 * ud, c2 * ud ** 2 + c1 * udd


# ============================================================================
# 3. CONTROLLER (paper)
# ============================================================================

def quat2rot(q):
    q0, q1, q2, q3 = q
    return np.array([[1 - 2 * (q2 ** 2 + q3 ** 2), 2 * (q1 * q2 - q0 * q3), 2 * (q0 * q2 + q1 * q3)],
                     [2 * (q1 * q2 + q0 * q3), 1 - 2 * (q1 ** 2 + q3 ** 2), 2 * (q2 * q3 - q0 * q1)],
                     [2 * (q1 * q3 - q0 * q2), 2 * (q0 * q1 + q2 * q3), 1 - 2 * (q1 ** 2 + q2 ** 2)]])  # eq. (2)


def Qmat(qb):
    """omega = Q(qbar) qbar_dot   (paper eq. 5)."""
    q0 = np.sqrt(max(1 - qb @ qb, 0.04))
    M = np.array([[q0, qb[2], -qb[1]], [-qb[2], q0, qb[0]], [qb[1], -qb[0], q0]])
    return 2 * (M + np.outer(qb, qb) / q0)


def flatness_map(a, q3r, m, g):
    """Desired acceleration -> total thrust and reference quaternion vector (eq. 16-17)."""
    N = max(np.linalg.norm([a[0], a[1], a[2] + g]), 0.3 * g)
    T = m * N
    q0 = np.sqrt(max(1 + (a[2] + g) / N - 2 * q3r ** 2, 0) / 2)
    den = max(N + a[2] + g, 1e-6)
    q1 = (a[0] * q3r - q0 * a[1]) / den
    q2 = (a[1] * q3r + q0 * a[0]) / den
    return T, np.array([q1, q2, q3r])


class PositionLoop:
    """50 Hz outer loop: xi_ddot* = xi_ddot_c + Kp e + Kd e' + Ki int(e)  (eq. 20)."""
    def __init__(self, cfg):
        self.cfg = cfg
        self.intE = np.zeros(3)
        self.Tc = cfg.m * cfg.g
        self.qr = np.zeros(3)

    def update(self, pc, vc, ac, ph, vh):
        c = self.cfg
        dtp = c.posLoopEvery * c.dt
        e, ed = pc - ph, vc - vh
        self.intE = np.clip(self.intE + e * dtp, -0.5, 0.5)
        a = ac + c.Kp * e + c.Kd * ed + c.Ki * self.intE
        axy = np.linalg.norm(a[:2])
        if axy > 4:
            a[:2] *= 4 / axy
        a[2] = max(a[2], -0.7 * c.g)
        T, self.qr = flatness_map(a, c.q3r, c.m, c.g)
        self.Tc = float(np.clip(T, 0, c.Tmax))
        return self.Tc, self.qr


class AttitudeLoop:
    """200 Hz inner loop: command filter (gives qr', qr'') + computed torque (eq. 26-27)."""
    def __init__(self, cfg):
        self.cfg = cfg
        self.qf = np.zeros(3); self.qfd = np.zeros(3); self.intEq = np.zeros(3)

    def update(self, qm, wm, qr):
        c = self.cfg
        wf = c.cmdFilterWn
        qfdd = wf ** 2 * (qr - self.qf) - 2 * wf * self.qfd
        self.qfd = self.qfd + c.dt * qfdd
        self.qf = self.qf + c.dt * self.qfd
        qb = qm[1:]
        qdm = 0.5 * np.r_[-qb @ wm, qm[0] * wm + np.cross(qb, wm)]      # q' = 0.5 q (x) w
        qbd = qdm[1:]
        eq, eqd = self.qf - qb, self.qfd - qbd
        self.intEq = np.clip(self.intEq + eq * c.dt, -0.2, 0.2)
        tp = qfdd + c.Kpq * eq + c.Kdq * eqd + c.Kiq * self.intEq
        Qm = Qmat(qb)
        eps = 1e-6
        Qdot_qd = (Qmat(qb + eps * qbd) - Qm) / eps @ qbd
        wdot = Qm @ tp + Qdot_qd
        return c.J * wdot + np.cross(wm, c.J * wm)


# ============================================================================
# 4. ESTIMATION + LIDAR
# ============================================================================

class OwnStateKF:
    def __init__(self, cfg, p0):
        self.c = cfg
        self.x = np.r_[p0, 0, 0, 0]
        self.P = np.diag([1e-4] * 3 + [1e-2] * 3)

    def predict(self, a_imu):
        """Every step: integrate the accelerometer (prediction)."""
        dt, c = self.c.dt, self.c
        F = np.block([[np.eye(3), dt * np.eye(3)], [np.zeros((3, 3)), np.eye(3)]])
        self.x = F @ self.x + np.r_[0.5 * dt ** 2 * a_imu, dt * a_imu]
        G = np.vstack([0.5 * dt ** 2 * np.eye(3), dt * np.eye(3)])
        self.P = F @ self.P @ F.T + G @ (c.accStd ** 2 * np.eye(3)) @ G.T

    def update(self, z):
        """20 Hz: blend prediction and GPS:  x = x_pred + K (z - H x_pred)."""
        H = np.hstack([np.eye(3), np.zeros((3, 3))])
        S = H @ self.P @ H.T + self.c.gpsStd ** 2 * np.eye(3)
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K @ (z - H @ self.x)
        self.P = (np.eye(6) - K @ H) @ self.P


def fibonacci_dirs(n):
    i = np.arange(n)
    z = 1 - 2 * (i + 0.5) / n
    r = np.sqrt(1 - z ** 2)
    th = np.pi * (3 - np.sqrt(5)) * i
    return np.stack([r * np.cos(th), r * np.sin(th), z], axis=1)


class Lidar:
    """3-D LiDAR: nRays rays over the whole sphere (up, down, left, right ...).
    Returns the set of obstacle body ids hit within sensorRange."""
    def __init__(self, cfg, client):
        self.c, self.cl = cfg, client
        self.dirs = fibonacci_dirs(cfg.nRays)
        self.last_hits = {}

    def scan(self, pos, obstacle_ids):
        c = self.c
        r0 = c.droneRadius + 0.02                          # start outside the drone's own body
        frm = pos + self.dirs * r0
        to = pos + self.dirs * c.sensorRange
        res = p.rayTestBatch(frm.tolist(), to.tolist(), physicsClientId=self.cl)
        hits = {}
        for (uid, _, frac, hp, hn) in res:
            if uid in obstacle_ids:
                hits.setdefault(uid, []).append(hp)
        self.last_hits = hits
        return hits


class ObstacleTrackers:
    """One constant-velocity Kalman filter per obstacle (buildings: static)."""
    def __init__(self, cfg, kinds, sizes):
        self.c, self.kinds, self.n = cfg, kinds, len(kinds)
        self.valid = np.zeros(self.n, bool)
        self.x = np.zeros((self.n, 6))
        self.P = np.zeros((self.n, 6, 6))
        self.seen = np.full(self.n, -np.inf)
        self.size = np.zeros((self.n, 3))
        self.true_sizes = sizes
        self.rng = None

    def step(self, t, detected, centers_true, rng):
        """detected[j] = lidar hit it.  Measurement = noisy centre (size assumed known once seen)."""
        c = self.c
        dts = c.sensorEvery * c.dt
        F = np.block([[np.eye(3), dts * np.eye(3)], [np.zeros((3, 3)), np.eye(3)]])
        H = np.hstack([np.eye(3), np.zeros((3, 3))])
        for j in range(self.n):
            static = self.kinds[j] == 'box'
            qv = 0.0 if static else 0.05
            Q = np.diag([1e-4] * 3 + [qv * dts] * 3)
            if self.valid[j]:
                self.x[j] = F @ self.x[j]
                if static:
                    self.x[j, 3:] = 0
                self.P[j] = F @ self.P[j] @ F.T + Q
            if detected[j]:
                z = centers_true[j] + c.sensorStd * rng.standard_normal(3)
                if not self.valid[j]:
                    self.x[j] = np.r_[z, 0, 0, 0]
                    vv = 1e-6 if static else 0.25
                    self.P[j] = np.diag([c.sensorStd ** 2] * 3 + [vv] * 3)
                    self.size[j] = self.true_sizes[j] + 0.01 * np.abs(rng.standard_normal(3))
                    self.valid[j] = True
                else:
                    R = c.sensorStd ** 2 * np.eye(3)
                    S = H @ self.P[j] @ H.T + R
                    K = self.P[j] @ H.T @ np.linalg.inv(S)
                    self.x[j] = self.x[j] + K @ (z - H @ self.x[j])
                    self.P[j] = (np.eye(6) - K @ H) @ self.P[j]
                self.seen[j] = t

    def active(self, t):
        return [dict(type=self.kinds[j], c=self.x[j, :3].copy(), v=self.x[j, 3:].copy(), size=self.size[j].copy())
                for j in range(self.n) if self.valid[j] and (t - self.seen[j]) < 1.5]


# ============================================================================
# 5. 3-D AVOIDANCE PLANNER
# ============================================================================

def clearance(P, typ, c, size):
    """Distance from points P (...,3) to the obstacle SURFACE (negative = inside)."""
    if typ == 'sphere':
        return np.linalg.norm(P - c, axis=-1) - size[0]
    q = np.abs(P - c) - size
    return np.linalg.norm(np.maximum(q, 0), axis=-1) + np.minimum(np.max(q, axis=-1), 0)


class Planner:
    def __init__(self, cfg):
        self.c = cfg
        wn, h = cfg.deltaWn, cfg.planStep
        K = int(round(cfg.planHorizon / h))
        A = np.array([[0, 1, 0], [0, 0, 1], [-wn ** 3, -3 * wn ** 2, -3 * wn]])
        B = np.array([0, 0, wn ** 3])
        Phi = expm(A * h)
        Gam = np.linalg.solve(A, Phi - np.eye(3)) @ B
        Ainit = np.zeros((K, 3)); b = np.zeros(K)
        Pk = np.eye(3); acc = np.zeros(3)
        for k in range(K):
            acc = Phi @ acc + Gam
            Pk = Phi @ Pk
            Ainit[k] = Pk[0]; b[k] = acc[0]
        dirs = fibonacci_dirs(cfg.nDirs)
        mags = np.linspace(cfg.maxOffset / cfg.nMags, cfg.maxOffset, cfg.nMags)
        self.Dc = np.vstack([np.zeros((1, 3))] + [m * dirs for m in mags])
        self.K, self.Ainit, self.b = K, Ainit, b
        self.tau = (np.arange(K) + 1) * h
        # detour filter state: rows = axis, cols = [d, d', d'']
        self.ds = np.zeros((3, 3))
        self.dstar = np.zeros(3)

    def plan(self, refpts, tracks):
        c = self.c
        dprev = self.dstar
        Dc = np.vstack([self.Dc[:1], dprev[None], self.Dc[1:]])         # row0 none, row1 previous
        Nc = len(Dc)
        D0 = self.Ainit @ self.ds.T                                     # K x 3 free response
        Pk = refpts[:, None, :] + D0[:, None, :] + self.b[:, None, None] * Dc[None]   # K x Nc x 3
        minClr = np.full(Nc, np.inf)
        for ob in tracks:
            cc = ob['c'][None, None, :] + ob['v'][None, None, :] * self.tau[:, None, None]
            cl = clearance(Pk, ob['type'], cc, ob['size']) - c.rho
            minClr = np.minimum(minClr, cl.min(axis=0))
        minClr = np.minimum(minClr, (Pk[..., 2] - c.zMin).min(axis=0))
        minClr = np.minimum(minClr, (c.zMax - Pk[..., 2]).min(axis=0))
        W = np.array([1, 1, c.wVert])
        cost = (Dc ** 2 * W).sum(1) + c.wPrev * ((Dc - dprev) ** 2).sum(1) + 1e-6 * np.arange(1, Nc + 1)
        cost[2:] += c.wSwitch                                           # switching costs extra
        cost[1] = (dprev ** 2 * W).sum()                                # keeping it is free
        allowed = minClr >= c.slack
        allowed[1] = minClr[1] >= 0
        if not allowed.any():
            allowed = minClr >= 0
        if allowed.any():
            cost[~allowed] = np.inf
            idx = int(np.argmin(cost))
        else:
            idx = int(np.argmax(minClr))                                # emergency: best effort
        self.dstar = Dc[idx].copy()
        return self.dstar

    def filter_step(self, dstar):
        """3rd-order detour filter, every step -> delta, delta', delta''."""
        c = self.c
        wn, dt = c.deltaWn, c.dt
        for i in range(3):
            d0, d1, d2 = self.ds[i]
            self.ds[i, 2] = d2 + dt * (wn ** 3 * (dstar[i] - d0) - 3 * wn ** 2 * d1 - 3 * wn * d2)
            self.ds[i, 1] = d1 + dt * d2
            self.ds[i, 0] = d0 + dt * d1
        return self.ds[:, 0].copy(), self.ds[:, 1].copy(), self.ds[:, 2].copy()


def label_decision(d, vref, thr=0.15):
    h = np.array([vref[0], vref[1], 0.0])
    h = h / np.linalg.norm(h) if np.linalg.norm(h) > 1e-6 else np.array([1.0, 0, 0])
    left = np.cross([0, 0, 1], h)
    lat, up = d @ left, d[2]
    parts = []
    if lat > thr: parts.append('LEFT')
    if lat < -thr: parts.append('RIGHT')
    if up > thr: parts.append('UP')
    if up < -thr: parts.append('DOWN')
    return '+'.join(parts) if parts else 'STRAIGHT'


# ============================================================================
# 6. PYBULLET WORLD
# ============================================================================

def _write_quad_obj(path):
    """Writes a small quadcopter mesh (hub, two crossing arms, four rotor discs) as an OBJ file."""
    V, F = [], []

    def box(cx, cy, cz, hx, hy, hz, rot=0.0):
        c, s = np.cos(rot), np.sin(rot)
        base = len(V) + 1
        for sx in (-1, 1):
            for sy in (-1, 1):
                for sz in (-1, 1):
                    x, y = sx * hx, sy * hy
                    V.append((cx + c * x - s * y, cy + s * x + c * y, cz + sz * hz))
        for a, b, cc, d in [(0, 1, 3, 2), (4, 6, 7, 5), (0, 4, 5, 1), (2, 3, 7, 6), (0, 2, 6, 4), (1, 5, 7, 3)]:
            F.append((base + a, base + b, base + cc)); F.append((base + a, base + cc, base + d))

    def disc(cx, cy, cz, r, n=14, h=0.004):
        base = len(V) + 1
        for k in range(n):
            a = 2 * np.pi * k / n
            V.append((cx + r * np.cos(a), cy + r * np.sin(a), cz - h)); V.append((cx + r * np.cos(a), cy + r * np.sin(a), cz + h))
        for k in range(n):
            a, b = 2 * k, (2 * k + 2) % (2 * n)
            F.append((base + a, base + b, base + a + 1)); F.append((base + a + 1, base + b, base + b + 1))
        for k in range(1, n - 1):
            F.append((base, base + 2 * k, base + 2 * (k + 1)))
            F.append((base + 1, base + 2 * (k + 1) + 1, base + 2 * k + 1))

    box(0, 0, 0, 0.045, 0.045, 0.02)
    box(0, 0, 0, 0.14, 0.008, 0.006, rot=np.pi / 4)
    box(0, 0, 0, 0.14, 0.008, 0.006, rot=-np.pi / 4)
    for sx, sy in [(1, 1), (1, -1), (-1, 1), (-1, -1)]:
        disc(sx * 0.1, sy * 0.1, 0.012, 0.055)
    with open(path, 'w') as f:
        for v in V:
            f.write('v %.5f %.5f %.5f\n' % v)
        for t in F:
            f.write('f %d %d %d\n' % t)


class World:
    def __init__(self, cfg, buildings, dyn, ref, p0, gui=False, visScale=1.6):
        self.cfg, self.gui = cfg, gui
        self.cl = p.connect(p.GUI if gui else p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath(), physicsClientId=self.cl)
        p.setGravity(0, 0, -cfg.g, physicsClientId=self.cl)
        p.setTimeStep(cfg.dt, physicsClientId=self.cl)
        if gui:
            p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 1, physicsClientId=self.cl)
        self.plane = p.loadURDF('plane.urdf', physicsClientId=self.cl)
        # ---- buildings (static boxes) ---------------------------------------
        self.ids, self.kinds, self.sizes = [], [], []
        cols = [[0.55, 0.60, 0.70, 0.9], [0.70, 0.62, 0.55, 0.9], [0.60, 0.68, 0.62, 0.9]]
        for i, b in enumerate(buildings):
            he = [b[2] / 2, b[3] / 2, (b[5] - b[4]) / 2]
            col = p.createCollisionShape(p.GEOM_BOX, halfExtents=he, physicsClientId=self.cl)
            vis = p.createVisualShape(p.GEOM_BOX, halfExtents=he, rgbaColor=cols[i % 3], physicsClientId=self.cl)
            uid = p.createMultiBody(0, col, vis, [b[0], b[1], (b[4] + b[5]) / 2], physicsClientId=self.cl)
            self.ids.append(uid); self.kinds.append('box'); self.sizes.append(np.array(he))
        self.nBuild = len(buildings)
        # ---- moving obstacles (kinematic spheres) ------------------------------
        self.dyn_p0, self.dyn_v = [], []
        for d in dyn:
            r = d[4]
            col = p.createCollisionShape(p.GEOM_SPHERE, radius=r, physicsClientId=self.cl)
            vis = p.createVisualShape(p.GEOM_SPHERE, radius=r, rgbaColor=[0.95, 0.4, 0.15, 1], physicsClientId=self.cl)
            uid = p.createMultiBody(0, col, vis, [0, 0, -50], physicsClientId=self.cl)
            pos_hit, _, _ = ref.eval(d[0])
            if len(d) >= 8: pos_hit = pos_hit + np.array(d[5:8])
            v = np.array(d[1:4])
            self.ids.append(uid); self.kinds.append('sphere'); self.sizes.append(np.array([r, 0, 0]))
            self.dyn_p0.append(pos_hit - v * d[0]); self.dyn_v.append(v)
        self.idset = set(self.ids)
        self.move_obstacles(0.0)
        self.centers0 = [np.array(b[:2].tolist() + [(b[4] + b[5]) / 2]) for b in buildings]
        # ---- quadcopter ---------------------------------------------------------
        obj = os.path.join(tempfile.gettempdir(), 'quad_visual.obj')
        _write_quad_obj(obj)
        vis = p.createVisualShape(p.GEOM_MESH, fileName=obj, meshScale=[visScale] * 3,
                                  rgbaColor=[0.15, 0.15, 0.18, 1], physicsClientId=self.cl)
        col = p.createCollisionShape(p.GEOM_SPHERE, radius=cfg.droneRadius, physicsClientId=self.cl)
        self.drone = p.createMultiBody(cfg.m, col, vis, list(p0), [0, 0, 0, 1], physicsClientId=self.cl)
        p.changeDynamics(self.drone, -1, localInertiaDiagonal=list(cfg.J), linearDamping=0, angularDamping=0,
                         restitution=0.1, physicsClientId=self.cl)

    # ------------------------------------------------------------------ helpers
    def move_obstacles(self, t):
        for k, (p0, v) in enumerate(zip(self.dyn_p0, self.dyn_v)):
            pos = p0 + v * t
            p.resetBasePositionAndOrientation(self.ids[self.nBuild + k], pos.tolist(), [0, 0, 0, 1], physicsClientId=self.cl)

    def obstacle_centers(self, t):
        c = list(self.centers0)
        c += [self.dyn_p0[k] + self.dyn_v[k] * t for k in range(len(self.dyn_p0))]
        return c

    def state(self):
        pos, orn = p.getBasePositionAndOrientation(self.drone, physicsClientId=self.cl)
        lin, ang = p.getBaseVelocity(self.drone, physicsClientId=self.cl)
        q = np.array([orn[3], orn[0], orn[1], orn[2]])               # PyBullet is (x,y,z,w) -> scalar first
        return np.array(pos), np.array(lin), q, np.array(ang)

    def collided(self):
        for c in p.getContactPoints(bodyA=self.drone, physicsClientId=self.cl):
            if c[2] in self.idset:
                return True
        return False

    def close(self):
        try:
            p.disconnect(self.cl)
        except Exception:
            pass


# ============================================================================
# 7. SIMULATION LOOP
# ============================================================================


def run(cfg=None, gui=False, rate=4.0, verbose=True):
    cfg = cfg or Config()
    if cfg.seed is None:
        cfg.seed = int(np.random.SeedSequence().entropy % 1000000)       # NEW random world every run
    rng = np.random.default_rng(cfg.seed)
    ref = Reference(cfg, rng)
    buildings, dyn = world_layout(cfg, ref)
    p0 = ref.eval(0.0)[0]
    pDest = ref.eval(cfg.Ttraj)[0]
    W = World(cfg, buildings, dyn, ref, p0, gui=gui)
    nO = len(W.ids)
    if verbose:
        _print_scenario(cfg, buildings, dyn, ref, W)
    dt, g, m = cfg.dt, cfg.g, cfg.m
    N = int((cfg.Ttraj + cfg.tHold) / dt) + 1

    kf = OwnStateKF(cfg, p0)
    lidar = Lidar(cfg, W.cl)
    trk = ObstacleTrackers(cfg, W.kinds, W.sizes)
    planner = Planner(cfg)
    posL, attL = PositionLoop(cfg), AttitudeLoop(cfg)
    wind = np.zeros(3)
    decision, events = 'STRAIGHT', []

    # ---- logs --------------------------------------------------------------------
    L = {k: np.zeros((N, n)) for k, n in dict(p=3, v=3, q=4, w=3, tau=3, ph=3, vh=3, pr=3, pc=3, delta=3,
                                              qref=4, wind=3, gps=3).items()}
    L['gps'][:] = np.nan
    L['t'] = np.zeros(N); L['T'] = np.zeros(N)
    L['obsPos'] = np.zeros((N, nO, 3)); L['clr'] = np.zeros((N, nO)); L['det'] = np.zeros((N, nO), bool)
    L['trkPos'] = np.full((N, nO, 3), np.nan)
    L['decision'] = [''] * N
    collisions = 0

    # ---- GUI extras -------------------------------------------------------------------
    gui_state = None
    if gui:
        gui_state = _gui_setup(cfg, ref, W, pDest, rate)

    v_prev = np.zeros(3); R_prev = np.eye(3); qm_prev = np.array([1.0, 0, 0, 0])
    a_true_prev = np.zeros(3)
    vr_now = np.zeros(3)
    k = 0
    skip = False
    while k < N:
        if gui:
            action = _gui_poll(gui_state)
            if action == 'pause':
                time.sleep(0.05); continue
            skip = skip or action == 'skip'
        t = k * dt
        pos, v, q, w_world = W.state()
        R = quat2rot(q)
        w = R.T @ w_world                                           # body angular rate

        # ---- sensors ---------------------------------------------------------------
        qm = q + cfg.attStd * rng.standard_normal(4); qm /= np.linalg.norm(qm)
        if qm[0] < 0: qm = -qm
        wm = w + cfg.gyroStd * rng.standard_normal(3)
        if k > 0:                                                    # IMU -> Kalman PREDICT (previous step)
            f_body = R_prev.T @ (a_true_prev + np.array([0, 0, g])) + cfg.accStd * rng.standard_normal(3)
            a_imu = quat2rot(qm_prev / np.linalg.norm(qm_prev)) @ f_body + np.array([0, 0, -g])
            kf.predict(a_imu)
        gps = np.full(3, np.nan)
        centers = W.obstacle_centers(t)
        det = np.zeros(nO, bool)
        if k % cfg.sensorEvery == 0:
            gps = pos + cfg.gpsStd * rng.standard_normal(3)
            kf.update(gps)                                           # Kalman CORRECT with GPS
            hits = lidar.scan(pos, W.idset)                          # 3-D LiDAR (PyBullet rays)
            det = np.array([W.ids[j] in hits for j in range(nO)])
            trk.step(t, det, centers, rng)
        ph, vh = kf.x[:3].copy(), kf.x[3:].copy()

        # ---- avoidance planner (10 Hz) ---------------------------------------------------
        if k % cfg.planEvery == 0:
            if cfg.enableAvoidance:
                refpts = np.array([ref.eval(t + (i + 1) * cfg.planStep)[0] for i in range(planner.K)])
                planner.plan(refpts, trk.active(t))
                vr_now = ref.eval(t)[1]
                new = label_decision(planner.ds[:, 0], vr_now)
                if new != decision:
                    events.append('t=%5.1f s : %s  (detour %.2f m)' % (t, new, np.linalg.norm(planner.dstar)))
                    if verbose and cfg.printEvery:
                        print('   >>> t=%5.1f s  DECISION: %-10s  (detour target %.2f m)' % (t, new, np.linalg.norm(planner.dstar)), flush=True)
                    decision = new
            else:
                planner.dstar = np.zeros(3)
        delta, deltad, deltadd = planner.filter_step(planner.dstar)

        # ---- controller ------------------------------------------------------------------
        pr, vr, ar = ref.eval(t)
        pc, vc, ac = pr + delta, vr + deltad, ar + deltadd
        if k % cfg.posLoopEvery == 0:
            posL.update(pc, vc, ac, ph, vh)
        tau = attL.update(qm, wm, posL.qr)
        Tc = posL.Tc

        # ---- disturbances ------------------------------------------------------------------
        wind = wind - wind * dt + cfg.windStd * np.sqrt(2 * dt) * rng.standard_normal(3)
        Fg = np.zeros(3)
        g0 = cfg.gust
        if g0[0] <= t < g0[0] + g0[1]:
            Fg = np.array(g0[2:5])
        Fext = wind + Fg
        Fdrag = -0.5 * cfg.rho_air * cfg.CD * cfg.A * np.linalg.norm(v) * v

        # ---- logging (state at the START of the step) ----------------------------------------
        L['t'][k] = t; L['p'][k] = pos; L['v'][k] = v; L['q'][k] = q; L['w'][k] = w
        L['T'][k] = Tc; L['tau'][k] = tau; L['ph'][k] = ph; L['vh'][k] = vh
        L['pr'][k] = pr; L['pc'][k] = pc; L['delta'][k] = delta
        L['qref'][k] = np.r_[np.sqrt(max(1 - attL.qf @ attL.qf, 0)), attL.qf]
        L['wind'][k] = Fext; L['gps'][k] = gps; L['decision'][k] = decision
        for j in range(nO):
            L['obsPos'][k, j] = centers[j]
            L['clr'][k, j] = clearance(pos, W.kinds[j], centers[j], W.sizes[j]) - cfg.droneRadius
            L['det'][k, j] = (L['clr'][k, j] + cfg.droneRadius) <= cfg.sensorRange
            if trk.valid[j]:
                L['trkPos'][k, j] = trk.x[j, :3]

        # ---- live terminal table: actual coordinates vs original reference path -------------------------
        if verbose and cfg.printEvery and k % max(1, int(round(cfg.printEvery / dt))) == 0:
            if k == 0:
                print('\n  time |   ACTUAL drone position (m)   |  ORIGINAL reference path (m)  | deviation | detour | min clr | state')
                print('   (s) |      x        y        z      |      x        y        z      |    (m)    |  (m)   |   (m)   |')
            dev = np.linalg.norm(pos - pr); dn = np.linalg.norm(delta)
            print('%6.1f | %8.3f %8.3f %8.3f | %8.3f %8.3f %8.3f | %8.3f  | %5.2f  | %6.2f  | %s' % (
                t, pos[0], pos[1], pos[2], pr[0], pr[1], pr[2], dev, dn, L['clr'][k].min(),
                decision if dn > 0.15 else 'on reference path'), flush=True)

        # ---- apply thrust + torque to the PyBullet body, step the physics --------------------
        F = R @ np.array([0, 0, Tc]) + Fdrag + Fext
        p.applyExternalForce(W.drone, -1, F.tolist(), pos.tolist(), p.WORLD_FRAME, physicsClientId=W.cl)
        p.applyExternalTorque(W.drone, -1, (R @ tau).tolist(), p.WORLD_FRAME, physicsClientId=W.cl)
        W.move_obstacles(t + dt)
        p.stepSimulation(physicsClientId=W.cl)
        if W.collided():
            collisions += 1
        v_new = W.state()[1]
        a_true_prev = (v_new - v) / dt
        R_prev, qm_prev = R, qm

        if gui:
            _gui_draw(gui_state, W, k, t, pos, delta, decision, L, trk, det, centers, cfg)
            if not skip:
                time.sleep(max(0.0, dt / max(gui_state['speed'], 0.05) - 0.0004))
        k += 1

    res = dict(cfg=cfg, ref=ref, buildings=buildings, dyn=dyn, L=L, kinds=W.kinds, sizes=W.sizes,
               pDest=pDest, nObs=nO, events=events, collisionSteps=collisions, world=W)
    _metrics(res)
    if verbose:
        summary(res)
    return res


def _metrics(res):
    L, cfg = res['L'], res['cfg']
    res['minClr'] = L['clr'].min(axis=0)
    res['rmse_cmd'] = float(np.sqrt(np.mean(((L['p'] - L['pc']) ** 2).sum(1))))
    ok = ~np.isnan(L['gps'][:, 0])
    res['rmse_gps'] = float(np.sqrt(np.mean(((L['gps'][ok] - L['p'][ok]) ** 2).sum(1))))
    res['rmse_kf'] = float(np.sqrt(np.mean(((L['ph'] - L['p']) ** 2).sum(1))))
    res['finalErr'] = float(np.linalg.norm(L['p'][-1] - res['pDest']))
    res['maxDelta'] = float(np.linalg.norm(L['delta'], axis=1).max())
    res['collided'] = bool(res['collisionSteps'] > 0 or (res['minClr'] < 0).any())


def summary(res):
    cfg = res['cfg']; names = obstacle_names(res)
    print('\n================ RESULT ================')
    print('required clearance from surfaces (beyond drone body): %.2f m' % cfg.margin)
    for j, n in enumerate(names):
        print('  %-12s min clearance %+.3f m' % (n, res['minClr'][j]))
    print('PyBullet contact steps with obstacles : %d' % res['collisionSteps'])
    print('final distance to destination         : %.3f m' % res['finalErr'])
    print('tracking RMSE (actual vs commanded)   : %.3f m' % res['rmse_cmd'])
    print('own-position error  GPS %.3f m -> Kalman %.3f m' % (res['rmse_gps'], res['rmse_kf']))
    print('max detour from original path         : %.2f m' % res['maxDelta'])
    print('decisions:')
    for e in res['events']:
        print('   ' + e)


def _print_scenario(cfg, buildings, dyn, ref, W):
    print('\n=========== SCENARIO (random seed %d  ->  repeat it with --seed %d) ===========' % (cfg.seed, cfg.seed))
    print('reference: noisy B-spline, degree %d, %d control points, start (%.2f, %.2f, %.2f) -> destination (%.2f, %.2f, %.2f)' % (
        cfg.bsplineDegree, cfg.nCtrl, *ref.eval(0)[0], *ref.eval(cfg.Ttraj)[0]))
    for i, b in enumerate(buildings):
        print('  Building %s: centre (%.2f, %.2f)  size %.2f x %.2f   z from %.2f to %.2f m' % ('ABCDEFGH'[i], b[0], b[1], b[2], b[3], b[4], b[5]))
    for i, d in enumerate(dyn):
        hit = ref.eval(d[0])[0] + (d[5:8] if len(d) >= 8 else 0)
        print('  Moving D%d : radius %.2f m, velocity (%.2f, %.2f, %.2f) m/s, passes the path near (%.2f, %.2f, %.2f) at t = %.1f s' % (
            i + 1, d[4], d[1], d[2], d[3], *hit, d[0]))
    print('The drone does NOT know any of this in advance: it only sees what its LiDAR hits (range %.1f m).' % cfg.sensorRange)


def obstacle_names(res):
    nb = sum(1 for k in res['kinds'] if k == 'box')
    return ['Building %s' % 'ABCDEFGH'[i] for i in range(nb)] + ['Moving D%d' % (i + 1) for i in range(len(res['kinds']) - nb)]


# ====================================================================== GUI helpers
def _gui_setup(cfg, ref, W, pDest, rate):
    cl = W.cl
    cur = ref.curve(120)
    for a, b in zip(cur[:-1], cur[1:]):                              # dashed-looking grey reference
        p.addUserDebugLine(a.tolist(), b.tolist(), [0.45, 0.45, 0.45], 1.5, 0, physicsClientId=cl)
    for pt, col in [(ref.eval(0)[0], [0.1, 0.8, 0.1, 1]), (pDest, [1, 0, 0, 1])]:
        vs = p.createVisualShape(p.GEOM_SPHERE, radius=0.18, rgbaColor=col, physicsClientId=cl)
        p.createMultiBody(0, -1, vs, pt.tolist(), physicsClientId=cl)
    for i in range(ref.waypoints.shape[1]):
        p.addUserDebugText('P%d' % i, (ref.waypoints[:, i] + [0, 0, 0.35]).tolist(), [0, 0, 0], 1.2, physicsClientId=cl)
    mid = ((ref.eval(0)[0] + pDest) / 2).tolist()
    p.resetDebugVisualizerCamera(11.0, 40, -28, mid, physicsClientId=cl)
    return dict(
        btn_pause=p.addUserDebugParameter('Pause / Resume', 1, 0, 0, physicsClientId=cl),
        btn_skip=p.addUserDebugParameter('Skip to end', 1, 0, 0, physicsClientId=cl),
        sld_speed=p.addUserDebugParameter('Speed x', 0.25, 16, rate, physicsClientId=cl),
        btn_follow=p.addUserDebugParameter('Follow camera', 1, 0, 0, physicsClientId=cl),
        last=dict(pause=0, skip=0, follow=0), paused=False, speed=rate, follow=False,
        trail_prev=None, det_lines=[-1] * len(W.ids), hud=-1, cl=cl, ring=[-1] * 3, camset=0)


def _gui_poll(S):
    cl = S['cl']
    S['speed'] = p.readUserDebugParameter(S['sld_speed'], physicsClientId=cl)
    v = p.readUserDebugParameter(S['btn_pause'], physicsClientId=cl)
    if v != S['last']['pause']:
        S['last']['pause'] = v; S['paused'] = not S['paused']
    keys = p.getKeyboardEvents(physicsClientId=cl)
    if 32 in keys and keys[32] & p.KEY_WAS_TRIGGERED:                # SPACE
        S['paused'] = not S['paused']
    v = p.readUserDebugParameter(S['btn_follow'], physicsClientId=cl)
    if v != S['last']['follow']:
        S['last']['follow'] = v; S['follow'] = not S['follow']
    v = p.readUserDebugParameter(S['btn_skip'], physicsClientId=cl)
    if v != S['last']['skip']:
        S['last']['skip'] = v
        return 'skip'
    return 'pause' if S['paused'] else None


def _gui_draw(S, W, k, t, pos, delta, decision, L, trk, det, centers, cfg):
    cl = S['cl']
    if k % 10 == 0:                                                  # every 0.05 s: keep ALL visited points
        if S['trail_prev'] is not None:
            p.addUserDebugLine(S['trail_prev'].tolist(), pos.tolist(), [0.1, 0.3, 1.0], 3.5, 0, physicsClientId=cl)
        S['trail_prev'] = pos.copy()
        for j in range(len(W.ids)):                                  # red line to every obstacle in lidar range
            if L['det'][k, j] and trk.valid[j]:
                S['det_lines'][j] = p.addUserDebugLine(pos.tolist(), trk.x[j, :3].tolist(), [1, 0, 0], 2.5, 0.12,
                                                       replaceItemUniqueId=S['det_lines'][j], physicsClientId=cl)
        clr = L['clr'][k].min()
        txt = 't=%5.1f s   detour %.2f m   min clearance %.2f m   %s' % (
            t, np.linalg.norm(delta), clr, decision if np.linalg.norm(delta) > 0.15 else 'ON REFERENCE PATH')
        S['hud'] = p.addUserDebugText(txt, (pos + [0, 0, 0.9]).tolist(), [0, 0, 0], 1.3, 0.12,
                                      replaceItemUniqueId=S['hud'], physicsClientId=cl)
        if S['follow']:
            p.resetDebugVisualizerCamera(6.0, 40, -25, pos.tolist(), physicsClientId=cl)


# ============================================================================
# 8. PLOTS
# ============================================================================
C = plt.rcParams['axes.prop_cycle'].by_key()['color']


def _shade(ax, t, mask):
    d = np.diff(np.r_[0, mask.astype(int), 0])
    for s, e in zip(np.where(d == 1)[0], np.where(d == -1)[0] - 1):
        ax.axvspan(t[s], t[e], color=(0.85, 0.88, 0.95), alpha=0.6, lw=0)


def _box(ax, b):
    x0, x1, y0, y1, z0, z1 = b[0] - b[2] / 2, b[0] + b[2] / 2, b[1] - b[3] / 2, b[1] + b[3] / 2, b[4], b[5]
    V = np.array([[x0, y0, z0], [x1, y0, z0], [x1, y1, z0], [x0, y1, z0], [x0, y0, z1], [x1, y0, z1], [x1, y1, z1], [x0, y1, z1]])
    faces = [[0, 1, 2, 3], [4, 5, 6, 7], [0, 1, 5, 4], [1, 2, 6, 5], [2, 3, 7, 6], [3, 0, 4, 7]]
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection
    ax.add_collection3d(Poly3DCollection([V[f] for f in faces], facecolor=(0.6, 0.65, 0.75, 0.35), edgecolor='k', lw=0.5))


def _equal3d(ax, P):
    mid = (P.max(0) + P.min(0)) / 2
    r = (P.max(0) - P.min(0)).max() / 2 + 0.5
    ax.set_xlim(mid[0] - r, mid[0] + r); ax.set_ylim(mid[1] - r, mid[1] + r); ax.set_zlim(max(0, mid[2] - r), mid[2] + r)


def make_plots(res, outdir='results', show=True):
    cfg, L, ref = res['cfg'], res['L'], res['ref']
    t = L['t']; names = obstacle_names(res); nO = res['nObs']
    active = np.linalg.norm(L['delta'], axis=1) > 0.15
    figs = {}
    os.makedirs(outdir, exist_ok=True)

    # 1 reference generation
    f = plt.figure('1 Reference path generation', figsize=(9, 7)); ax = f.add_subplot(111, projection='3d')
    cur = ref.curve(400)
    ax.plot(*ref.raw.T, '.', color='0.6', ms=4, label='noisy raw samples (σ=%.2f m)' % cfg.refNoiseStd)
    ax.plot(*ref.ideal.T, 'k:', lw=1, label='ideal polyline')
    ax.plot(*cur.T, lw=2.5, label='degree-%d B-spline (used as reference)' % cfg.bsplineDegree)
    ax.plot(*ref.C0.T, 's--', color=C[1], label='%d control points' % cfg.nCtrl)
    ax.plot(*ref.waypoints, 'p', color='k', mfc='gold', ms=11, label='waypoints P0-P6')
    ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)'); ax.set_zlabel('Z (m)'); ax.legend(); ax.set_title('Reference path generation')
    figs['1_reference_bspline'] = f

    # 2 x,y,z tracking
    f, axs = plt.subplots(3, 1, sharex=True, figsize=(10, 8)); f.canvas.manager.set_window_title('2 Position tracking x,y,z')
    for i, lab in enumerate(['x (m)', 'y (m)', 'z (m)']):
        _shade(axs[i], t, active)
        axs[i].plot(t, L['pr'][:, i], '--', color='0.45', label='nominal reference')
        axs[i].plot(t, L['pc'][:, i], color='#4d9be6', lw=1.2, label='commanded (reference + detour)')
        axs[i].plot(t, L['p'][:, i], color='#d9401a', lw=1.6, label='actual (PyBullet)')
        axs[i].set_ylabel(lab); axs[i].grid(alpha=.3)
    axs[0].legend(ncol=3, fontsize=8); axs[0].set_title('Position tracking (shaded = avoidance in progress)'); axs[2].set_xlabel('time (s)')
    figs['2_xyz_tracking'] = f

    # 3 3-D tracking
    f = plt.figure('3 Quadcopter 3-D tracking', figsize=(10, 8)); ax = f.add_subplot(111, projection='3d')
    zcap = L['p'][:, 2].max() + 1.5                           # draw tall towers only up to the flight height
    for b in res['buildings']:
        b = np.array(b, float); b[5] = min(b[5], zcap)
        _box(ax, b)
    for j in range(nO):
        if res['kinds'][j] == 'sphere':
            ax.plot(*L['obsPos'][:, j].T, ':', color='#e6733a', lw=1)
            k = int(len(t) * 0.55); r = res['sizes'][j][0]
            u, v = np.mgrid[0:2 * np.pi:14j, 0:np.pi:8j]
            c = L['obsPos'][k, j]
            ax.plot_surface(c[0] + r * np.cos(u) * np.sin(v), c[1] + r * np.sin(u) * np.sin(v), c[2] + r * np.cos(v), color='#f2733f', alpha=.7)
    ax.plot(*L['pr'].T, '--', color='0.4', lw=1.8, label='nominal reference')
    ax.plot(*L['p'].T, color='#d9401a', lw=2, label='actual path')
    sk = slice(0, None, max(1, int(round(0.05 / cfg.dt))))
    ax.plot(*L['p'][sk].T, '.', color='#1a5fe6', ms=4, label='visited points (every 0.05 s)')
    ax.plot(*L['p'][0:1].T, 'o', color='g', ms=10, label='start'); ax.plot(*res['pDest'][:, None], 'p', color='r', ms=14, label='destination')
    _equal3d(ax, np.vstack([L['p'], L['pr']])); ax.view_init(24, 38)
    ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)'); ax.set_zlabel('Z (m)'); ax.legend(); ax.set_title('3-D tracking: every visited point')
    figs['3_trajectory_3D'] = f

    # 4 thrust + torque
    f, axs = plt.subplots(4, 1, sharex=True, figsize=(10, 9)); f.canvas.manager.set_window_title('4 Total thrust and control torque')
    _shade(axs[0], t, active); axs[0].plot(t, L['T'], color=C[0]); axs[0].axhline(cfg.m * cfg.g, color='k', ls=':', label='weight mg')
    axs[0].set_ylabel('thrust T (N)'); axs[0].legend(); axs[0].set_title('Total thrust and control torques')
    for i, lab in enumerate(['τx (N m)', 'τy (N m)', 'τz (N m)']):
        _shade(axs[i + 1], t, active); axs[i + 1].plot(t, L['tau'][:, i], color=C[i + 1]); axs[i + 1].set_ylabel(lab)
    for a in axs: a.grid(alpha=.3)
    axs[3].set_xlabel('time (s)'); figs['4_thrust_torque'] = f

    # 5 angular velocity
    f, axs = plt.subplots(3, 1, sharex=True, figsize=(10, 7)); f.canvas.manager.set_window_title('5 Angular velocity')
    for i, lab in enumerate(['ωx (rad/s)', 'ωy (rad/s)', 'ωz (rad/s)']):
        _shade(axs[i], t, active); axs[i].plot(t, L['w'][:, i], color=C[i]); axs[i].set_ylabel(lab); axs[i].grid(alpha=.3)
    axs[0].set_title('Body angular velocity'); axs[2].set_xlabel('time (s)'); figs['5_angular_velocity'] = f

    # 6 attitude
    f, axs = plt.subplots(4, 2, figsize=(12, 9)); f.canvas.manager.set_window_title('6 Attitude: quaternion and Euler angles')
    for i in range(4):
        a = axs[i, 0]; _shade(a, t, active)
        a.plot(t, L['qref'][:, i], '--', color='0.4', label='reference'); a.plot(t, L['q'][:, i], color=C[i], label='actual')
        a.set_ylabel('q%d' % i); a.grid(alpha=.3)
    axs[0, 0].set_title('Quaternion: reference vs actual'); axs[0, 0].legend(fontsize=8)
    q = L['q']
    roll = np.arctan2(2 * (q[:, 0] * q[:, 1] + q[:, 2] * q[:, 3]), 1 - 2 * (q[:, 1] ** 2 + q[:, 2] ** 2))
    pitch = np.arcsin(np.clip(2 * (q[:, 0] * q[:, 2] - q[:, 1] * q[:, 3]), -1, 1))
    yaw = np.arctan2(2 * (q[:, 0] * q[:, 3] + q[:, 1] * q[:, 2]), 1 - 2 * (q[:, 2] ** 2 + q[:, 3] ** 2))
    for i, (e, lab) in enumerate(zip([roll, pitch, yaw], ['roll φ (deg)', 'pitch θ (deg)', 'yaw ψ (deg)'])):
        axs[i, 1].plot(t, np.degrees(e), color=C[i + 4]); axs[i, 1].set_ylabel(lab); axs[i, 1].grid(alpha=.3)
    axs[0, 1].set_title('Euler angles'); axs[3, 1].axis('off'); axs[3, 0].set_xlabel('time (s)'); axs[2, 1].set_xlabel('time (s)')
    figs['6_attitude'] = f

    # 7 avoidance
    f, axs = plt.subplots(3, 1, sharex=True, figsize=(10, 9)); f.canvas.manager.set_window_title('7 Obstacle avoidance')
    for i, lab in enumerate(['δx', 'δy', 'δz']): axs[0].plot(t, L['delta'][:, i], label=lab)
    axs[0].plot(t, np.linalg.norm(L['delta'], axis=1), 'k--', label='|δ|'); axs[0].legend(ncol=4); axs[0].set_ylabel('detour (m)')
    axs[0].set_title('Detour added to the reference (back to 0 = back on the original path)')
    for j in range(nO): axs[1].plot(t, L['clr'][:, j], label=names[j])
    axs[1].axhline(0, color='r', lw=1.5, label='collision (0)'); axs[1].set_ylim(-0.1, 6); axs[1].legend(ncol=3, fontsize=8)
    axs[1].set_ylabel('clearance (m)'); axs[1].set_title('Distance from the drone body to every obstacle surface')
    axs[2].plot(t, np.linalg.norm(L['v'], axis=1), label='speed'); axs[2].set_ylabel('speed (m/s)'); axs[2].set_xlabel('time (s)')
    a2 = axs[2].twinx(); a2.step(t, L['det'].sum(1), color=C[1], label='obstacles in sensor range'); a2.set_ylabel('count'); a2.legend(loc='upper right')
    for a in axs: a.grid(alpha=.3)
    figs['7_avoidance'] = f

    # 8 Kalman blending
    f, axs = plt.subplots(2, 1, sharex=True, figsize=(10, 7)); f.canvas.manager.set_window_title('8 Kalman filter blending')
    ok = ~np.isnan(L['gps'][:, 0])
    axs[0].plot(t[ok], np.linalg.norm(L['gps'][ok] - L['p'][ok], axis=1), '.', color='#cc6633', label='raw GPS (RMS %.3f m)' % res['rmse_gps'])
    axs[0].plot(t, np.linalg.norm(L['ph'] - L['p'], axis=1), color='#1a66cc', label='Kalman = IMU prediction blended with GPS (RMS %.3f m)' % res['rmse_kf'])
    axs[0].set_ylabel('position error (m)'); axs[0].legend(); axs[0].set_title('Own-position estimate')
    for j in range(nO):
        e = np.linalg.norm(L['trkPos'][:, j] - L['obsPos'][:, j], axis=1); axs[1].plot(t, e, label=names[j])
    axs[1].set_ylabel('obstacle track error (m)'); axs[1].set_xlabel('time (s)'); axs[1].legend(ncol=3, fontsize=8)
    for a in axs: a.grid(alpha=.3)
    figs['8_kalman'] = f

    for name, fig in figs.items():
        fig.tight_layout(); fig.savefig(os.path.join(outdir, name + '.png'), dpi=110)
    pts = np.column_stack([t, L['p'], L['pr'], np.linalg.norm(L['p'] - L['pr'], axis=1), np.linalg.norm(L['delta'], axis=1),
                           np.linalg.norm(L['v'], axis=1)])[::max(1, int(round(0.05 / cfg.dt)))]
    np.savetxt(os.path.join(outdir, 'travelled_points.csv'), pts, delimiter=',', comments='', fmt='%.4f',
               header='time_s,x_m,y_m,z_m,ref_x_m,ref_y_m,ref_z_m,deviation_m,detour_m,speed_mps')
    print('Saved %d figures and travelled_points.csv in %s/' % (len(figs), outdir))
    if show:
        plt.show()
    return figs


# ============================================================================
# 9. OFFLINE TUNING (learn)
# ============================================================================
BOUNDS = dict(margin=(0.10, 0.30), deltaWn=(1.5, 3.2), wPrev=(0.3, 1.0), wSwitch=(0.0, 0.25), planHorizon=(1.8, 3.2))


def score(res):
    cfg, L = res['cfg'], res['L']
    J = 1000.0 if res['collided'] else 0.0
    J += 200 * max(0.0, -res['minClr'].min())
    J += 20 * res['finalErr'] + 2 * res['maxDelta']
    a = np.diff(L['v'], axis=0) / cfg.dt
    J += 0.5 * np.sqrt(np.mean((a ** 2).sum(1)))
    return J


def learn_main(n_trials=12, seeds=(1, 2, 3)):
    rng = np.random.default_rng(123)
    best, bestJ = None, np.inf
    for n in range(n_trials):
        th = {k: (getattr(Config(), k) if n == 0 else rng.uniform(*v)) for k, v in BOUNDS.items()}   # trial 0 = current
        J = 0.0
        for s in seeds:
            cfg = Config()
            for k, v in th.items(): setattr(cfg, k, v)
            cfg.seed = 1000 + s
            cfg.dyn_speed_scale = 0.8 + 0.4 * rng.random()
            res = run(cfg, gui=False, verbose=False)
            res['world'].close()
            J += score(res)
        J /= len(seeds)
        print('trial %2d  %s  -> cost %.2f' % (n + 1, {k: round(v, 2) for k, v in th.items()}, J))
        if J < bestJ:
            best, bestJ = th, J
    print('\nBest parameters (cost %.2f):' % bestJ)
    for k, v in best.items():
        print('    self.%s = %.3f' % (k, v))




# ============================================================================
# 10. MAIN
# ============================================================================
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--no-gui', action='store_true')
    ap.add_argument('--rate', type=float, default=4.0, help='playback speed')
    ap.add_argument('--no-avoid', action='store_true', help='baseline: shows the collisions')
    ap.add_argument('--compare', action='store_true', help='also run once WITHOUT avoidance (same world) and print both')
    ap.add_argument('--no-show', action='store_true')
    ap.add_argument('--seed', type=int, default=None, help='repeat a scenario (default: new random scenario each run)')
    ap.add_argument('--print-every', type=float, default=0.5, help='terminal table interval in seconds (0 = off)')
    ap.add_argument('--learn', nargs='*', type=int, help='N_TRIALS [seeds...]  offline tuning')
    a = ap.parse_args()
    if a.learn is not None:
        learn_main(a.learn[0] if a.learn else 12, tuple(a.learn[1:]) or (1, 2, 3))
        return
    cfg = Config()
    cfg.enableAvoidance = not a.no_avoid
    cfg.seed = a.seed
    cfg.printEvery = a.print_every
    base = None
    if a.compare:
        c0 = Config(); c0.enableAvoidance = False; c0.printEvery = 0
        cfg.seed = c0.seed = a.seed if a.seed is not None else int(time.time()) % 1000000   # same world for both runs
        print('--- baseline (avoidance OFF) ---')
        base = run(c0, gui=False, verbose=True)
        base['world'].close()
    res = run(cfg, gui=not a.no_gui, rate=a.rate)
    if base is not None:
        print('\nmin clearance WITHOUT avoidance:', ['%+.2f' % x for x in base['minClr']])
        print('min clearance WITH    avoidance:', ['%+.2f' % x for x in res['minClr']])
    make_plots(res, 'results', show=not (a.no_gui or a.no_show))
    res['world'].close()


if __name__ == '__main__':
    main()
