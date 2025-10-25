#!/usr/bin/env python3
"""
Path smoothing utilities using B-splines.

Provides functions to smooth 2D waypoints and compute curvature.
"""

from typing import List, Tuple

import numpy as np
from scipy.interpolate import splev, splprep


def smooth_path(
    waypoints: List[Tuple[float, float]],
    smoothness: float = 0.35,
    num_points: int = 400,
) -> List[Tuple[float, float, float]]:
    """
    Smooth a sequence of 2D waypoints using a B-spline and compute curvature.

    Args:
        waypoints (List[Tuple[float, float]]): List of (x, y) waypoints.
        smoothness (float): Spline smoothing factor (higher = smoother).
        num_points (int): Number of output points along the spline.

    Returns:
        List[Tuple[float, float, float]]: List of (x, y, kappa) where kappa
        is the curvature at that point. Returns an empty list for empty input.
        Robust to degenerate inputs (1 or 2 points).
    """
    if not waypoints:
        return []

    n = len(waypoints)

    # Handle single-point input
    if n == 1:
        x, y = waypoints[0]
        return [(float(x), float(y), 0.0)]

    # Handle two-point input: linear interpolation
    if n == 2:
        x0, y0 = waypoints[0]
        x1, y1 = waypoints[1]
        t = np.linspace(0, 1, num_points)
        xs = x0 + (x1 - x0) * t
        ys = y0 + (y1 - y0) * t
        return [(float(x), float(y), 0.0) for x, y in zip(xs, ys)]

    # General case: n >= 3
    x = [p[0] for p in waypoints]
    y = [p[1] for p in waypoints]
    k = min(3, n - 1)  # cubic B-spline unless n < 4

    try:
        # Fit B-spline
        tck, u = splprep([x, y], s=smoothness, k=k)

        # Evaluate spline at dense points
        u_fine = np.linspace(0, 1, num_points)
        xs, ys = splev(u_fine, tck)
        dx, dy = splev(u_fine, tck, der=1)
        ddx, ddy = splev(u_fine, tck, der=2)

        result: List[Tuple[float, float, float]] = []
        for xi, yi, x1, y1, x2, y2 in zip(xs, ys, dx, dy, ddx, ddy):
            denom = (x1 * x1 + y1 * y1) ** 1.5
            kappa = float((x1 * y2 - y1 * x2) / denom) if denom != 0 else 0.0
            result.append((float(xi), float(yi), kappa))
        return result

    except Exception:
        # Fallback: linear dense interpolation for robustness
        pts: List[Tuple[float, float, float]] = []
        segments = max(1, n - 1)
        samples_per_seg = max(2, int(num_points / segments))

        for i in range(1, n):
            a = np.array(waypoints[i - 1])
            b = np.array(waypoints[i])
            for t in np.linspace(0, 1, samples_per_seg, endpoint=False):
                p = a * (1 - t) + b * t
                pts.append((float(p[0]), float(p[1]), 0.0))

        # Append final waypoint
        pts.append((float(waypoints[-1][0]), float(waypoints[-1][1]), 0.0))
        return pts

