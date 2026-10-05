"""Tests for general QCar longitudinal and localization safeguards."""

import sys
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "qcar"))

from control_safety import (  # noqa: E402
    LocalizationFreshnessMonitor,
    SafeSpeedController,
    TrackingLossMonitor,
)


class SafeSpeedControllerTests(unittest.TestCase):
    def test_stall_does_not_wind_integral_beyond_limit(self):
        controller = SafeSpeedController()
        output = 0.0
        for _ in range(1000):
            output = controller.update(0.0, 0.25, 0.01)
        self.assertLessEqual(controller.integral_error, 0.05)
        self.assertLessEqual(output, 0.15)

    def test_moving_target_includes_static_friction_compensation(self):
        controller = SafeSpeedController(static_feedforward=0.05)
        output = controller.update(0.0, 0.25, 0.01)
        self.assertGreater(output, 0.0)
        self.assertLessEqual(output, 0.15)

    def test_slowdown_target_retains_enough_bounded_drive_effort(self):
        controller = SafeSpeedController(static_feedforward=0.05)
        output = 0.0
        for _ in range(100):
            output = controller.update(0.0, 0.0625, 0.01)
        self.assertGreaterEqual(output, 0.09)
        self.assertLessEqual(output, 0.15)

    def test_stop_has_no_static_feedforward(self):
        controller = SafeSpeedController(static_feedforward=0.05)
        output = controller.update(0.0, 0.0, 0.01)
        self.assertEqual(output, 0.0)
        self.assertEqual(controller.state, "STOPPED")

    def test_overspeed_coasts_and_clears_positive_integral(self):
        controller = SafeSpeedController()
        for _ in range(100):
            controller.update(0.0, 0.25, 0.01)
        output = controller.update(0.7, 0.25, 0.01)
        self.assertEqual(controller.state, "EMERGENCY_OVERSPEED_COAST")
        self.assertEqual(output, 0.0)
        self.assertLessEqual(controller.integral_error, 0.0)

    def test_stop_command_coasts_without_reverse_torque(self):
        controller = SafeSpeedController()
        output = controller.update(0.25, 0.0, 0.01)
        self.assertEqual(controller.state, "COASTING_TO_STOP")
        self.assertEqual(output, 0.0)

    def test_negative_tach_speed_does_not_request_extra_forward_throttle(self):
        controller = SafeSpeedController()
        output = controller.update(-0.70, 0.25, 0.01)
        self.assertEqual(controller.state, "EMERGENCY_OVERSPEED_COAST")
        self.assertEqual(output, 0.0)

    def test_throttle_command_has_bounded_slew_and_never_reverses(self):
        controller = SafeSpeedController(
            throttle_rise_rate_per_s=0.12,
            throttle_fall_rate_per_s=0.30,
        )
        previous = 0.0
        for _ in range(200):
            output = controller.update(0.0, 0.25, 0.02)
            self.assertGreaterEqual(output, 0.0)
            self.assertLessEqual(output - previous, 0.12 * 0.02 + 1e-9)
            previous = output

    def test_simulated_drive_is_smooth_and_tracks_cruise_speed(self):
        """Offline acceptance test with friction and first-order inertia."""
        controller = SafeSpeedController()
        speed = 0.0
        commands = []
        speeds = []
        dt = 0.02
        for _ in range(int(20.0 / dt)):
            command = controller.update(speed, 0.25, dt)
            # Simple conservative drivetrain: commands below 0.04 cannot
            # overcome static friction; above it, speed approaches a bounded
            # motor equilibrium with first-order inertia.
            equilibrium = max(0.0, 4.0 * (command - 0.04))
            speed += dt * (equilibrium - speed) / 0.45
            commands.append(command)
            speeds.append(speed)
        self.assertTrue(all(command >= 0.0 for command in commands))
        self.assertLessEqual(
            max(abs(b - a) for a, b in zip(commands, commands[1:])),
            0.30 * dt + 1e-9,
        )
        steady = speeds[-int(3.0 / dt):]
        self.assertLess(abs(sum(steady) / len(steady) - 0.25), 0.04)
        self.assertLess(max(steady) - min(steady), 0.04)


class LocalizationFreshnessMonitorTests(unittest.TestCase):
    def test_stale_pose_stops_and_requires_fresh_recovery(self):
        monitor = LocalizationFreshnessMonitor(
            stale_after_s=0.75, recover_below_s=0.25
        )
        stale = monitor.update(0.80)
        held = monitor.update(0.40)
        recovered = monitor.update(0.10)
        self.assertEqual(stale["speed_gain"], 0.0)
        self.assertEqual(held["state"], "STALE")
        self.assertEqual(recovered["state"], "FRESH")


class TrackingLossMonitorTests(unittest.TestCase):
    def test_transient_error_does_not_stop(self):
        monitor = TrackingLossMonitor(persistence_s=0.25)
        result = monitor.update(0.40, 0.0, 0.10)
        result = monitor.update(0.05, 0.0, 0.10)
        self.assertEqual(result["state"], "TRACKING")
        self.assertEqual(result["speed_gain"], 1.0)

    def test_sustained_large_error_latches_stop(self):
        monitor = TrackingLossMonitor(persistence_s=0.25)
        for _ in range(3):
            result = monitor.update(0.40, 0.0, 0.10)
        self.assertEqual(result["state"], "LOST")
        self.assertEqual(result["speed_gain"], 0.0)
        recovered = monitor.update(0.0, 0.0, 1.0)
        self.assertEqual(recovered["state"], "LOST")


if __name__ == "__main__":
    unittest.main()
