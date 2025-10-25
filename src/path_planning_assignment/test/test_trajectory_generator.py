#!/usr/bin/env python3
"""
Unit tests for trajectory generation module.
Tests velocity profiling, time parameterization, and physics constraints.
"""

import unittest
import math
from path_planning_assignment.trajectory_generator import generate_trajectory


class TestTrajectoryGeneration(unittest.TestCase):
    """Test suite for trajectory generation functionality."""

    def test_empty_path(self):
        """Test that empty path returns empty trajectory."""
        result = generate_trajectory([])
        self.assertEqual(len(result), 0)

    def test_single_point_trajectory(self):
        """Test single point trajectory generation."""
        path = [(1.0, 2.0, 0.0)]
        result = generate_trajectory(path)
        
        self.assertEqual(len(result), 1)
        self.assertAlmostEqual(result[0]['x'], 1.0, places=5)
        self.assertAlmostEqual(result[0]['y'], 2.0, places=5)
        self.assertAlmostEqual(result[0]['t'], 0.0, places=5)
        self.assertGreaterEqual(result[0]['v'], 0.0)

    def test_trajectory_output_format(self):
        """Test that output has correct dictionary format."""
        path = [
            (0.0, 0.0, 0.0),
            (1.0, 0.0, 0.0),
            (2.0, 0.0, 0.0)
        ]
        result = generate_trajectory(path)
        
        required_keys = ['x', 'y', 'kappa', 's', 'v', 't']
        for point in result:
            for key in required_keys:
                self.assertIn(key, point)
                self.assertIsInstance(point[key], float)

    def test_arc_length_computation(self):
        """Test that arc length 's' is computed correctly."""
        path = [
            (0.0, 0.0, 0.0),
            (1.0, 0.0, 0.0),
            (2.0, 0.0, 0.0),
            (3.0, 0.0, 0.0)
        ]
        result = generate_trajectory(path)
        
        # Check arc length increases monotonically
        for i in range(1, len(result)):
            self.assertGreaterEqual(result[i]['s'], result[i-1]['s'])
        
        # Total arc length should be approximately 3.0
        self.assertAlmostEqual(result[-1]['s'], 3.0, delta=0.1)

    def test_time_monotonicity(self):
        """Test that time increases monotonically."""
        path = [
            (0.0, 0.0, 0.0),
            (1.0, 1.0, 0.1),
            (2.0, 2.0, 0.05),
            (3.0, 3.0, 0.0)
        ]
        result = generate_trajectory(path)
        
        # Time should start at 0
        self.assertAlmostEqual(result[0]['t'], 0.0, places=5)
        
        # Time should increase monotonically
        for i in range(1, len(result)):
            self.assertGreater(result[i]['t'], result[i-1]['t'])

    def test_velocity_constraints(self):
        """Test that velocities respect maximum velocity constraint."""
        path = [
            (0.0, 0.0, 0.0),
            (1.0, 0.0, 0.0),
            (2.0, 0.0, 0.0)
        ]
        v_max = 0.5
        result = generate_trajectory(path, v_max=v_max)
        
        for point in result:
            self.assertLessEqual(point['v'], v_max + 0.01)  # Small tolerance
            self.assertGreaterEqual(point['v'], 0.0)

    def test_curvature_speed_reduction(self):
        """Test that high curvature reduces speed."""
        # Straight path (low curvature)
        straight_path = [
            (0.0, 0.0, 0.0),
            (1.0, 0.0, 0.0),
            (2.0, 0.0, 0.0)
        ]
        
        # Curved path (high curvature)
        curved_path = [
            (0.0, 0.0, 2.0),  # High curvature
            (1.0, 0.0, 2.0),
            (2.0, 0.0, 2.0)
        ]
        
        straight_traj = generate_trajectory(straight_path, v_max=0.5)
        curved_traj = generate_trajectory(curved_path, v_max=0.5)
        
        # Average velocity should be lower for curved path
        avg_v_straight = sum(p['v'] for p in straight_traj) / len(straight_traj)
        avg_v_curved = sum(p['v'] for p in curved_traj) / len(curved_traj)
        
        self.assertGreater(avg_v_straight, avg_v_curved, "Straight path velocity should be higher than curved path.")

    def test_acceleration_limits_forward(self):
        """Test forward pass acceleration limiting."""
        path = [
            (0.0, 0.0, 0.0),
            (1.0, 0.0, 0.0),
            (2.0, 0.0, 0.0),
            (3.0, 0.0, 0.0)
        ]
        a_max = 0.3
        result = generate_trajectory(path, a_max=a_max)
        
        # Check acceleration between consecutive points
        for i in range(1, len(result)):
            dt = result[i]['t'] - result[i-1]['t']
            if dt > 0:
                dv = result[i]['v'] - result[i-1]['v']
                accel = dv / dt
                # Allow some tolerance for numerical errors
                self.assertLessEqual(accel, a_max + 0.1)

    def test_deceleration_limits_backward(self):
        """Test backward pass deceleration limiting."""
        path = [
            (0.0, 0.0, 0.0),
            (1.0, 0.0, 0.0),
            (2.0, 0.0, 1.5),  # High curvature ahead forces deceleration
            (3.0, 0.0, 1.5)
        ]
        a_max = 0.4
        result = generate_trajectory(path, a_max=a_max)
        
        # Velocity should decrease before high curvature section
        self.assertLess(result[1]['v'], result[0]['v'] + 0.1)

    def test_velocity_profile_trapezoidal(self):
        """Test that velocity profile resembles trapezoidal shape."""
        # Long straight path
        path = [(float(i), 0.0, 0.0) for i in range(20)]
        result = generate_trajectory(path, v_max=0.5, a_max=0.4)
        
        velocities = [p['v'] for p in result]
        
        # Should accelerate at start
        self.assertLess(velocities[0], velocities[5], "Velocity should be increasing during acceleration phase.")
        
        # Should have constant velocity in middle (or close to max)
        mid_velocities = velocities[len(velocities)//3:2*len(velocities)//3]
        avg_mid = sum(mid_velocities) / len(mid_velocities)
        self.assertGreater(avg_mid, 0.3, "Middle velocities should be close to the maximum velocity.")

    def test_zero_curvature_max_speed(self):
        """Test that zero curvature allows maximum speed."""
        path = [
            (0.0, 0.0, 0.0),
            (1.0, 0.0, 0.0),
            (2.0, 0.0, 0.0)
        ]
        v_max = 0.8
        result = generate_trajectory(path, v_max=v_max, a_max=1.0)
        
        # At least some points should reach near v_max
        max_v = max(p['v'] for p in result)
        self.assertGreater(max_v, v_max * 0.7, f"Maximum velocity should be above {v_max * 0.7}.")

    def test_consistency_across_runs(self):
        """Test that trajectory generation is deterministic."""
        path = [
            (0.0, 0.0, 0.1),
            (1.0, 1.0, 0.2),
            (2.0, 0.0, 0.1)
        ]
        
        result1 = generate_trajectory(path, v_max=0.5, a_max=0.5)
        result2 = generate_trajectory(path, v_max=0.5, a_max=0.5)
        
        self.assertEqual(len(result1), len(result2))
        for p1, p2 in zip(result1, result2):
            self.assertAlmostEqual(p1['v'], p2['v'], places=10)
            self.assertAlmostEqual(p1['t'], p2['t'], places=10)
            self.assertAlmostEqual(p1['s'], p2['s'], places=10)

if __name__ == '__main__':
    unittest.main()
