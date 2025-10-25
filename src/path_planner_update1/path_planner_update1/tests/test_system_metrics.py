import math

import numpy as np

from path_planner_update1.controller import AdvancedController
from path_planner_update1.path_smoothing import smooth_path
from path_planner_update1.trajectory_generator import generate_trajectory


def simulate_tracking(num_steps=150):
    waypoints = [(0, 0), (1, 1), (2, 0)]
    path = smooth_path(waypoints, smoothness=0.3, num_points=200)
    traj = generate_trajectory(path, v_max=0.2)
    ctrl = AdvancedController()
    pose = [0.0, 0.0, 0.0]
    dt = 0.1
    errors = []
    for step in range(num_steps):
        v, w, st = ctrl.step(tuple(pose), traj)
        # simple integrator
        pose[0] += v * math.cos(pose[2]) * dt
        pose[1] += v * math.sin(pose[2]) * dt
        pose[2] += w * dt
        target = traj[min(step, len(traj) - 1)]
        errors.append(math.hypot(target["x"] - pose[0], target["y"] - pose[1]))
    return np.mean(errors), np.max(errors)


def test_tracking_performance():
    mean_err, max_err = simulate_tracking()
    assert mean_err < 1.5
    assert max_err < 1.7
