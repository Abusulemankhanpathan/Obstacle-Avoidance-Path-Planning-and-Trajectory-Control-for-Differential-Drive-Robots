import time

from path_planner_update1.controller import AdvancedController


def test_controller_outputs():
    ctrl = AdvancedController()
    traj = [
        {"x": x, "y": 0.0, "kappa": 0.0, "s": x, "v": 0.2, "t": x / 0.2}
        for x in [0.0, 0.5, 1.0, 1.5]
    ]
    pose = (0.0, 0.1, 0.0)
    v, w, st = ctrl.step(pose, traj, time.time())
    assert v >= 0.0
    assert abs(w) < 2.0
    assert st in ("ok", "finished", "no_traj")


def test_rate_limiting():
    ctrl = AdvancedController(max_dv_per_sec=0.5)
    traj = [
        {"x": i, "y": 0.0, "kappa": 0.0, "s": i, "v": 0.2, "t": i / 0.2}
        for i in [0, 1, 2]
    ]
    pose = (0.0, 0.0, 0.0)
    t0 = time.time()
    v1, w1, st1 = ctrl.step(pose, traj, t0)
    v2, w2, st2 = ctrl.step(pose, traj, t0 + 0.05)
    assert abs(v2 - v1) <= 0.05 * 0.5 + 1e-6
