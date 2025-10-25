#!/usr/bin/env python3
"""
Trajectory generation utilities for 2D paths.

Generates a time-parameterized trajectory from a smoothed path with curvature.
Includes arc-length parameterization, curvature-aware speed profile, and timestamps.
"""

from typing import Dict, List, Tuple
import numpy as np


def generate_trajectory(
    path_with_kappa: List[Tuple[float, float, float]],
    v_max: float = 0.22,
    a_max: float = 0.6,
) -> List[Dict]:
    """
    Generate a trajectory with velocity and time stamps from a smoothed path.

    Args:
        path_with_kappa (List[Tuple[float, float, float]]): List of (x, y, kappa) points.
        v_max (float): Maximum forward velocity [m/s].
        a_max (float): Maximum acceleration [m/s^2].

    Returns:
        List[Dict]: Trajectory points with keys:
            'x', 'y', 'kappa', 's', 'v', 't'
    """
    if not path_with_kappa:
        return []

    # Compute cumulative arc-length s along path
    s_vals = [0.0]
    for i in range(1, len(path_with_kappa)):
        x0, y0, _ = path_with_kappa[i - 1]
        x1, y1, _ = path_with_kappa[i]
        s_vals.append(s_vals[-1] + float(np.hypot(x1 - x0, y1 - y0)))

    # Initialize trajectory points
    traj = []
    for (x, y, kappa), s in zip(path_with_kappa, s_vals):
        traj.append({
            "x": float(x),
            "y": float(y),
            "kappa": abs(float(kappa)),  # store absolute curvature
            "s": float(s),
            "v": 0.0,  # will compute below
            "t": 0.0,  # will compute below
        })

    # Curvature-based speed limit heuristic (adjusted)
    for pt in traj:
        kappa = pt["kappa"]
        if kappa < 1e-6:
            v_limit = v_max
        else:
            # Adjusted curvature-based velocity reduction
            v_limit = min(v_max, max(0.05, (1 / (kappa + 1e-6)) ** 0.5))
        pt["v"] = v_limit

    # Forward pass: acceleration limit
    for i in range(1, len(traj)):
        ds = traj[i]["s"] - traj[i - 1]["s"]
        v_prev = traj[i - 1]["v"]
        v_allow = (v_prev**2 + 2 * a_max * ds) ** 0.5
        traj[i]["v"] = min(traj[i]["v"], v_allow)

    # Backward pass: deceleration limit
    for i in range(len(traj) - 2, -1, -1):
        ds = traj[i + 1]["s"] - traj[i]["s"]
        v_next = traj[i + 1]["v"]
        v_allow = (v_next**2 + 2 * a_max * ds) ** 0.5
        traj[i]["v"] = min(traj[i]["v"], v_allow)

    # Compute timestamps
    t = 0.0
    traj[0]["t"] = 0.0
    for i in range(1, len(traj)):
        ds = max(1e-6, traj[i]["s"] - traj[i - 1]["s"])
        v_mid = max(1e-6, 0.5 * (traj[i]["v"] + traj[i - 1]["v"]))
        dt = ds / v_mid
        t += dt
        traj[i]["t"] = t

    return traj
