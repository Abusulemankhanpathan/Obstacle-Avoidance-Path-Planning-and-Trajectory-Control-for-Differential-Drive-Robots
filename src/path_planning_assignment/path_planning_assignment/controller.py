#!/usr/bin/env python3
"""
Advanced controller: curvature feedforward + PID heading compensation + rate limiting.
Simplified for assignment: removes obstacle recovery and wiggle logic.
"""

import math
import time


class PID:
    """PID controller with anti-windup and derivative smoothing."""

    def __init__(self, kp, ki=0.0, kd=0.0, windup=1.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.windup = windup
        self.integral = 0.0
        self.prev_err = None
        self.prev_t = None

    def reset(self):
        self.integral = 0.0
        self.prev_err = None
        self.prev_t = None

    def step(self, err, t):
        dt = 1e-6 if (self.prev_t is None) else max(1e-6, t - self.prev_t)
        self.integral += err * dt
        self.integral = max(-self.windup, min(self.windup, self.integral))
        deriv = 0.0 if self.prev_err is None else (err - self.prev_err) / dt
        deriv = deriv * 0.8 + 0.2 * deriv
        out = self.kp * err + self.ki * self.integral + self.kd * deriv
        self.prev_err = err
        self.prev_t = t
        return out


class AdvancedController:
    """Trajectory-following controller with curvature feedforward and PID."""

    def __init__(self, lookahead=0.6, v_max=0.22, angular_pid=(1.8, 0.0, 0.04), max_w=1.2, alpha=0.45, max_dv_per_sec=0.6):
        self.lookahead = float(lookahead)
        self.v_max = float(v_max)
        self.pid = PID(*angular_pid, windup=1.5)
        self.max_w = float(max_w)
        self.alpha = float(alpha)
        self.last_v = 0.0
        self.last_w = 0.0
        self.max_dv_per_sec = float(max_dv_per_sec)
        self.last_time = time.time()

    def find_lookahead_index(self, pose, traj):
        if not traj:
            return 0
        x, y, _ = pose
        dists = [math.hypot(pt["x"] - x, pt["y"] - y) for pt in traj]
        nearest_idx = int(min(range(len(dists)), key=lambda i: dists[i]))
        cum = 0.0
        idx = nearest_idx
        while idx + 1 < len(traj) and cum < self.lookahead:
            p0 = traj[idx]
            p1 = traj[idx + 1]
            cum += math.hypot(p1["x"] - p0["x"], p1["y"] - p0["y"])
            idx += 1
        if idx < nearest_idx:
            idx = nearest_idx
        return min(idx, len(traj) - 1)

    def step(self, pose, traj, now=None):
        if now is None:
            now = time.time()
        if not traj:
            return 0.0, 0.0, "no_traj"

        x, y, yaw = pose
        goal = traj[-1]
        dist_to_goal = math.hypot(goal["x"] - x, goal["y"] - y)
        if dist_to_goal < 0.10:
            self.pid.reset()
            self.last_v = 0.0
            self.last_w = 0.0
            return 0.0, 0.0, "finished"

        idx = self.find_lookahead_index(pose, traj)
        pt = traj[idx]

        angle_to_target = math.atan2(pt["y"] - y, pt["x"] - x)
        angle_err = math.atan2(math.sin(angle_to_target - yaw), math.cos(angle_to_target - yaw))

        kappa = pt.get("kappa", 0.0)
        v_ref = min(self.v_max, pt.get("v", self.v_max))
        w_ff = v_ref * kappa

        pid_out = self.pid.step(angle_err, now)
        w_cmd = w_ff + pid_out
        if abs(w_cmd) < 0.01:
            w_cmd = 0.0
        w_cmd = max(-self.max_w, min(self.max_w, w_cmd))

        heading_scale = max(0.0, math.cos(angle_err)) ** 0.9
        v_cmd = v_ref * heading_scale
        if abs(angle_err) > (math.pi / 2):
            v_cmd = min(v_cmd, 0.02)

        dt = max(1e-6, now - self.last_time)
        max_dv = self.max_dv_per_sec * dt
        dv = v_cmd - self.last_v
        if dv > max_dv:
            v_cmd = self.last_v + max_dv
        elif dv < -max_dv:
            v_cmd = self.last_v - max_dv

        v_out = self.alpha * v_cmd + (1.0 - self.alpha) * self.last_v
        w_out = self.alpha * w_cmd + (1.0 - self.alpha) * self.last_w

        v_out = max(-self.v_max, min(self.v_max, v_out))
        w_out = max(-self.max_w, min(self.max_w, w_out))

        self.last_v = v_out
        self.last_w = w_out
        self.last_time = now

        return float(v_out), float(w_out), "ok"
