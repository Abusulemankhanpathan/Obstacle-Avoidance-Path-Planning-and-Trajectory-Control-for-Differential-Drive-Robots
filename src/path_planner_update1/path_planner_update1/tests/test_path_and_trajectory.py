import math

from path_planner_update1.path_smoothing import smooth_path
from path_planner_update1.trajectory_generator import generate_trajectory


def test_smooth_nonempty():
    waypoints = [(0, 0), (1, 1), (2, 0)]
    p = smooth_path(waypoints, smoothness=0.3, num_points=50)
    assert len(p) > 10
    for x, y, k in p:
        assert math.isfinite(x) and math.isfinite(y)
        assert abs(k) < 20


def test_trajectory_profile():
    waypoints = [(0, 0), (1, 0), (2, 0)]
    p = smooth_path(waypoints, smoothness=0.1, num_points=100)
    traj = generate_trajectory(p, v_max=0.2, a_max=0.5)
    assert len(traj) > 0
    assert all(t["v"] >= 0.0 for t in traj)
    assert traj[-1]["t"] > 0.0
