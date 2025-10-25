#!/usr/bin/env python3
"""
Unit tests for trajectory tracking controller.
Tests PID controller, lookahead mechanism, and control outputs.
"""

import unittest
import math
import time
from path_planning_assignment.controller import PID, AdvancedController


class TestPIDController(unittest.TestCase):
    """Test suite for PID controller."""

    def test_pid_initialization(self):
        """Test PID controller initialization."""
        pid = PID(kp=1.0, ki=0.1, kd=0.05, windup=1.0)
        self.assertEqual(pid.kp, 1.0)
        self.assertEqual(pid.ki, 0.1)
        self.assertEqual(pid.kd, 0.05)
        self.assertEqual(pid.windup, 1.0)
        self.assertEqual(pid.integral, 0.0)

    def test_pid_proportional_only(self):
        """Test proportional-only control."""
        pid = PID(kp=2.0, ki=0.0, kd=0.0)
        error = 0.5
        output = pid.step(error, time.time())
        self.assertAlmostEqual(output, 1.0, places=5)  # 2.0 * 0.5

    def test_pid_integral_accumulation(self):
        """Test integral term accumulation."""
        pid = PID(kp=1.0, ki=0.5, kd=0.0)
        t = time.time()
        
        # Step with constant error
        output1 = pid.step(1.0, t)
        output2 = pid.step(1.0, t + 0.1)
        
        # Integral should accumulate, increasing output
        self.assertGreater(output2, output1)

    def test_pid_windup_limit(self):
        """Test anti-windup mechanism."""
        pid = PID(kp=0.0, ki=1.0, kd=0.0, windup=2.0)
        t = time.time()
        
        # Apply large error repeatedly
        for i in range(10):
            pid.step(10.0, t + i * 0.1)
        
        # Integral should be clamped
        self.assertLessEqual(abs(pid.integral), 2.0)

    def test_pid_derivative_term(self):
        """Test derivative term response to error change."""
        pid = PID(kp=0.0, ki=0.0, kd=1.0)
        t = time.time()
        
        # First step
        pid.step(0.0, t)
        
        # Second step with error change
        output = pid.step(1.0, t + 0.1)
        
        # Derivative should contribute (error changed from 0 to 1)
        self.assertGreater(abs(output), 0.0)

    def test_pid_reset(self):
        """Test PID reset functionality."""
        pid = PID(kp=1.0, ki=1.0, kd=1.0)
        t = time.time()
        
        # Accumulate state
        pid.step(1.0, t)
        pid.step(1.0, t + 0.1)
        
        # Reset
        pid.reset()
        
        self.assertEqual(pid.integral, 0.0)
        self.assertIsNone(pid.prev_err)
        self.assertIsNone(pid.prev_t)


class TestAdvancedController(unittest.TestCase):
    """Test suite for trajectory tracking controller."""

    def test_controller_initialization(self):
        """Test controller initialization with default parameters."""
        controller = AdvancedController()
        self.assertIsNotNone(controller.pid)
        self.assertEqual(controller.last_v, 0.0)
        self.assertEqual(controller.last_w, 0.0)

    def test_empty_trajectory(self):
        """Test controller behavior with empty trajectory."""
        controller = AdvancedController()
        pose = (0.0, 0.0, 0.0)
        v, w, status = controller.step(pose, [])
        
        self.assertEqual(v, 0.0)
        self.assertEqual(w, 0.0)
        self.assertEqual(status, "no_traj")

    def test_goal_reached(self):
        """Test controller stops when goal is reached."""
        controller = AdvancedController()
        pose = (1.0, 1.0, 0.0)
        trajectory = [
            {"x": 1.0, "y": 1.0, "kappa": 0.0, "v": 0.5}
        ]
        
        v, w, status = controller.step(pose, trajectory)
        
        self.assertEqual(v, 0.0)
        self.assertEqual(w, 0.0)
        self.assertEqual(status, "finished")

    def test_forward_motion_straight(self):
        """Test forward motion for straight trajectory."""
        controller = AdvancedController(v_max=0.5)
        pose = (0.0, 0.0, 0.0)  # Robot at origin, facing east
        trajectory = [
            {"x": 1.0, "y": 0.0, "kappa": 0.0, "v": 0.5},
            {"x": 2.0, "y": 0.0, "kappa": 0.0, "v": 0.5}
        ]
        
        v, w, status = controller.step(pose, trajectory)
        
        self.assertGreater(v, 0.0)  # Should move forward
        self.assertEqual(status, "ok")

    def test_angular_correction(self):
        """Test angular velocity for heading correction."""
        controller = AdvancedController()
        # Robot facing east, but goal is north
        pose = (0.0, 0.0, 0.0)
        trajectory = [
            {"x": 0.0, "y": 1.0, "kappa": 0.0, "v": 0.3}
        ]
        
        v, w, status = controller.step(pose, trajectory)
        
        # Should have positive angular velocity to turn left
        self.assertGreater(w, 0.0)
        self.assertEqual(status, "ok")

    def test_velocity_limits(self):
        """Test that velocities respect maximum constraints."""
        controller = AdvancedController(v_max=0.3, max_w=1.0)
        pose = (0.0, 0.0, 0.0)
        trajectory = [
            {"x": 5.0, "y": 5.0, "kappa": 0.0, "v": 1.0}  # Request high speed
        ]
        
        v, w, status = controller.step(pose, trajectory)
        
        self.assertLessEqual(abs(v), 0.3 + 0.01)  # Small tolerance
        self.assertLessEqual(abs(w), 1.0 + 0.01)

    def test_lookahead_mechanism(self):
        """Test lookahead point selection."""
        controller = AdvancedController(lookahead=1.0)
        pose = (0.0, 0.0, 0.0)
        
        # Create trajectory with multiple points
        trajectory = [
            {"x": 0.2, "y": 0.0, "kappa": 0.0, "v": 0.3},
            {"x": 0.5, "y": 0.0, "kappa": 0.0, "v": 0.3},
            {"x": 1.0, "y": 0.0, "kappa": 0.0, "v": 0.3},
            {"x": 2.0, "y": 0.0, "kappa": 0.0, "v": 0.3}
        ]
        
        idx = controller.find_lookahead_index(pose, trajectory)
        
        # Should select point approximately lookahead distance away
        self.assertGreaterEqual(idx, 0)
        self.assertLess(idx, len(trajectory))

    def test_curvature_feedforward(self):
        """Test curvature feedforward control."""
        controller = AdvancedController()
        pose = (0.0, 0.0, math.pi/2)  # Facing north
        
        # Curved trajectory
        trajectory = [
            {"x": 0.0, "y": 1.0, "kappa": 0.5, "v": 0.3}
        ]
        
        v, w, status = controller.step(pose, trajectory)
        
        # Should have angular velocity due to curvature
        self.assertNotEqual(w, 0.0)

    def test_rate_limiting(self):
        """Test velocity rate limiting (smooth acceleration)."""
        controller = AdvancedController(max_dv_per_sec=0.5)
        pose = (0.0, 0.0, 0.0)
        trajectory = [
            {"x": 5.0, "y": 0.0, "kappa": 0.0, "v": 0.5}
        ]
        
        t = time.time()
        v1, _, _ = controller.step(pose, trajectory, now=t)
        v2, _, _ = controller.step(pose, trajectory, now=t + 0.1)
        
        # Velocity change should be limited
        dv = abs(v2 - v1)
        max_allowed_dv = 0.5 * 0.1  # max_dv_per_sec * dt
        self.assertLessEqual(dv, max_allowed_dv)  # Fixed line
