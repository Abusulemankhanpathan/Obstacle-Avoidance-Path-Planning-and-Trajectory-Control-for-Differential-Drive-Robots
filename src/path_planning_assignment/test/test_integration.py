#!/usr/bin/env python3
"""
Integration tests for complete path planning pipeline.
Tests the full workflow: waypoints → smoothing → trajectory → control
"""

import unittest
import math
from path_planning_assignment.path_smoothing import smooth_path
from path_planning_assignment.trajectory_generator import generate_trajectory
from path_planning_assignment.controller import AdvancedController


class TestEndToEndPipeline(unittest.TestCase):
    """Test complete path planning workflow."""

    def test_simple_straight_path_pipeline(self):
        """Test complete pipeline with straight path."""
        # Step 1: Define waypoints
        waypoints = [
            (0.0, 0.0),
            (1.0, 0.0),
            (2.0, 0.0),
            (3.0, 0.0)
        ]
        
        # Step 2: Smooth path
        smoothed = smooth_path(waypoints, smoothness=0.3, num_points=100)
        self.assertEqual(len(smoothed), 100)
        
        # Step 3: Generate trajectory
        trajectory = generate_trajectory(smoothed, v_max=0.5, a_max=0.5)
        self.assertEqual(len(trajectory), 100)
        
        # Verify trajectory properties
        self.assertAlmostEqual(trajectory[0]['t'], 0.0, places=5)
        self.assertGreater(trajectory[-1]['t'], 0.0)
        
        # Step 4: Execute control
        controller = AdvancedController(v_max=0.5)
        pose = (0.0, 0.0, 0.0)
        v, w, status = controller.step(pose, trajectory)
        
        self.assertGreater(v, 0.0)
        self.assertEqual(status, "ok")

    def test_curved_path_pipeline(self):
        """Test pipeline with curved path."""
        # S-shaped curve
        waypoints = [
            (0.0, 0.0),
            (1.0, 1.0),
            (2.0, 1.0),
            (3.0, 0.0),
            (4.0, 0.0)
        ]
        
        smoothed = smooth_path(waypoints, smoothness=0.4, num_points=150)
        trajectory = generate_trajectory(smoothed, v_max=0.3, a_max=0.4)
        
        # Verify velocity reduces in curves
        curvatures = [abs(pt['kappa']) for pt in trajectory]
        velocities = [pt['v'] for pt in trajectory]
        
        # Find high curvature section
        max_curv_idx = curvatures.index(max(curvatures))
        v_at_curve = velocities[max_curv_idx]
        
        # Compare to straight section velocity
        min_curv_idx = curvatures.index(min(curvatures))
        v_at_straight = velocities[min_curv_idx]
        
        self.assertLess(v_at_curve, v_at_straight)

    def test_rectangular_path_tracking(self):
        """Test tracking rectangular path (typical robot task)."""
        waypoints = [
            (0.0, 0.0),
            (2.0, 0.0),
            (2.0, 2.0),
            (0.0, 2.0),
            (0.0, 0.0)
        ]
        
        smoothed = smooth_path(waypoints, smoothness=0.3, num_points=200)
        trajectory = generate_trajectory(smoothed, v_max=0.22, a_max=0.6)
        
        # Simulate tracking
        controller = AdvancedController(v_max=0.22, lookahead=0.6)
        
        # Start at first point
        pose = (smoothed[0][0], smoothed[0][1], 0.0)
        
        tracking_errors = []
        for _ in range(10):  # Simulate 10 control steps
            v, w, status = controller.step(pose, trajectory)
            
            # Simple motion model (assume perfect execution)
            dt = 0.05
            pose = (
                pose[0] + v * math.cos(pose[2]) * dt,
                pose[1] + v * math.sin(pose[2]) * dt,
                pose[2] + w * dt
            )
            
            # Compute tracking error
            target = trajectory[0]
            error = math.hypot(pose[0] - target['x'], pose[1] - target['y'])
            tracking_errors.append(error)
        
        # Error should decrease over time (converging to path)
        self.assertLess(tracking_errors[-1], tracking_errors[0] + 0.1)


class TestRobustnessAndEdgeCases(unittest.TestCase):
    """Test system robustness to edge cases."""

    def test_very_close_waypoints(self):
        """Test handling of very close waypoints."""
        waypoints = [
            (0.0, 0.0),
            (0.01, 0.01),
            (0.02, 0.01),
            (1.0, 1.0)
        ]
        
        smoothed = smooth_path(waypoints, num_points=100)
        trajectory = generate_trajectory(smoothed)
        
        self.assertGreater(len(smoothed), 0)
        self.assertGreater(len(trajectory), 0)

    def test_sharp_turn(self):
        """Test handling of sharp 90-degree turns."""
        waypoints = [
            (0.0, 0.0),
            (1.0, 0.0),
            (1.0, 1.0)
        ]
        
        smoothed = smooth_path(waypoints, smoothness=0.2, num_points=100)
        trajectory = generate_trajectory(smoothed, v_max=0.3, a_max=0.5)
        
        # Velocity should be reduced at the turn
        mid_point = trajectory[len(trajectory)//2]
        self.assertLess(mid_point['v'], 0.25)

    def test_long_distance_path(self):
        """Test system with long-distance path."""
        waypoints = [(float(i), 0.0) for i in range(20)]
        
        smoothed = smooth_path(waypoints, num_points=500)
        trajectory = generate_trajectory(smoothed, v_max=0.5)
        
        # Should generate reasonable trajectory
        self.assertEqual(len(trajectory), 500)
        self.assertGreater(trajectory[-1]['t'], 10.0)  # Should take time

    def test_controller_goal_reached(self):
        """Test controller behavior when goal is reached."""
        waypoints = [(0.0, 0.0), (0.05, 0.0)]
        smoothed = smooth_path(waypoints, num_points=10)
        trajectory = generate_trajectory(smoothed)
        
        controller = AdvancedController()
        
        # Start near goal
        pose = (0.04, 0.0, 0.0)
        v, w, status = controller.step(pose, trajectory)
        
        # Should recognize goal reached
        self.assertEqual(status, "finished")
        self.assertEqual(v, 0.0)
        self.assertEqual(w, 0.0)

    def test_backwards_facing_robot(self):
        """Test controller when robot faces away from path."""
        waypoints = [(0.0, 0.0), (1.0, 0.0)]
        smoothed = smooth_path(waypoints, num_points=50)
        trajectory = generate_trajectory(smoothed)
        
        controller = AdvancedController()
        
        # Robot facing backwards (180 degrees)
        pose = (0.0, 0.0, math.pi)
        v, w, status = controller.step(pose, trajectory)
        
        # Should turn to face goal
        self.assertNotEqual(w, 0.0)
        self.assertEqual(status, "ok")


class TestPerformanceAndOptimization(unittest.TestCase):
    """Test computational performance."""

    def test_smoothing_performance(self):
        """Test that smoothing completes in reasonable time."""
        import time
        
        waypoints = [(float(i), float(i % 2)) for i in range(50)]
        
        start = time.time()
        smoothed = smooth_path(waypoints, num_points=500)
        elapsed = time.time() - start
        
        self.assertLess(elapsed, 0.5)  # Should complete in <500ms
        self.assertEqual(len(smoothed), 500)

    def test_trajectory_generation_performance(self):
        """Test trajectory generation speed."""
        import time
        
        path = [(float(i), float(i % 2), 0.1) for i in range(500)]
        
        start = time.time()
        trajectory = generate_trajectory(path)
        elapsed = time.time() - start
        
        self.assertLess(elapsed, 0.2)  # Should complete in <200ms

    def test_control_loop_performance(self):
        """Test controller computation speed (critical for real-time)."""
        import time
        
        waypoints = [(0.0, 0.0), (5.0, 5.0)]
        smoothed = smooth_path(waypoints, num_points=100)
        trajectory = generate_trajectory(smoothed)
        
        controller = AdvancedController()
        pose = (0.0, 0.0, 0.0)
        
        # Measure control step time
        times = []
        for _ in range(100):
            start = time.time()
            controller.step(pose, trajectory)
            times.append(time.time() - start)
        
        avg_time = sum(times) / len(times)
        self.assertLess(avg_time, 0.01)  # <10ms for real-time (100Hz capable)


class TestDataConsistency(unittest.TestCase):
    """Test data integrity through pipeline."""

    def test_coordinate_consistency(self):
        """Test that coordinates remain consistent through pipeline."""
        waypoints = [
            (0.0, 0.0),
            (1.0, 1.0),
            (2.0, 0.0)
        ]
        
        smoothed = smooth_path(waypoints, num_points=100)
        trajectory = generate_trajectory(smoothed)
        
        # First and last points should be close to input waypoints
        self.assertAlmostEqual(smoothed[0][0], waypoints[0][0], delta=0.2)
        self.assertAlmostEqual(smoothed[0][1], waypoints[0][1], delta=0.2)
        self.assertAlmostEqual(smoothed[-1][0], waypoints[-1][0], delta=0.2)
        self.assertAlmostEqual(smoothed[-1][1], waypoints[-1][1], delta=0.2)
        
        # Trajectory should match smoothed path coordinates
        self.assertAlmostEqual(trajectory[0]['x'], smoothed[0][0], places=5)
        self.assertAlmostEqual(trajectory[0]['y'], smoothed[0][1], places=5)

    def test_time_stamps_valid(self):
        """Test that time stamps are valid and monotonic."""
        waypoints = [(float(i), 0.0) for i in range(10)]
        smoothed = smooth_path(waypoints, num_points=200)
        trajectory = generate_trajectory(smoothed)
        
        for i in range(1, len(trajectory)):
            # Time must be monotonically increasing
            self.assertGreater(trajectory[i]['t'], trajectory[i-1]['t'])
            
            # Arc length must be monotonically increasing
            self.assertGreaterEqual(trajectory[i]['s'], trajectory[i-1]['s'])

    def test_velocity_physical_feasibility(self):
        """Test that generated velocities are physically feasible."""
        waypoints = [(float(i), float(i % 2), 0.0) for i in range(20)]
        smoothed = smooth_path(waypoints, num_points=300)
        trajectory = generate_trajectory(smoothed, v_max=0.5, a_max=0.8)
        
        for i in range(1, len(trajectory)):
            # Check velocity limits
            self.assertGreaterEqual(trajectory[i]['v'], 0.0)
            self.assertLessEqual(trajectory[i]['v'], 0.5 + 0.01)
            
            # Check acceleration limits
            dt = trajectory[i]['t'] - trajectory[i-1]['t']
            if dt > 0:
                dv = trajectory[i]['v'] - trajectory[i-1]['v']
                accel = abs(dv / dt)
                self.assertLessEqual(accel, 0.8 + 0.2)  # Allow some tolerance


if __name__ == '__main__':
    unittest.main()
