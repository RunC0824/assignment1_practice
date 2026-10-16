"""Numerical and timestamp edge cases for the coursework localisation method."""
import math
import unittest
from ira1_sim.odometry import WheelOdometry, wrap_angle
from ira1_sim.evaluation import Pose, TimeAligner, interpolate_truth, pose_error


def ideal(**kwargs):
    return WheelOdometry(encoder_counts_per_rev=0, noise_std_m_per_sqrt_m=0.0,
                         left_scale_bias=0.0, right_scale_bias=0.0, **kwargs)


class OdometryTests(unittest.TestCase):
    def test_straight_and_in_place_turn(self):
        model = ideal()
        model.update(0., 0., 0.)
        sample = model.update(1., 1./.15, 1./.15)
        self.assertAlmostEqual(sample.x_m, 1.)
        self.assertAlmostEqual(sample.y_m, 0.)
        turn_wheel_angle = (.625*math.pi/4)/.15
        sample = model.update(2., 1./.15-turn_wheel_angle, 1./.15+turn_wheel_angle)
        self.assertAlmostEqual(sample.x_m, 1.)
        self.assertAlmostEqual(sample.y_m, 0.)
        self.assertAlmostEqual(sample.theta_rad, math.pi/2)

    def test_lecture_heading_error_example(self):
        model = ideal()
        model.update(0., 0., 0.)
        angle = math.radians(2.)
        sample = model.update(1., (1.-.625*angle/2)/.15, (1.+.625*angle/2)/.15)
        self.assertAlmostEqual(sample.x_m, math.cos(math.radians(1.)))
        self.assertAlmostEqual(sample.y_m, math.sin(math.radians(1.)))
        self.assertAlmostEqual(sample.theta_rad, angle)

    def test_short_intervals_preserve_accumulated_wheel_motion(self):
        model = ideal()
        model.update(0., 0., 0.)
        self.assertIsNone(model.update(.01, .5, .5))
        sample = model.update(.02, 1., 1.)
        self.assertAlmostEqual(sample.x_m, .15)

    def test_quantised_encoder_uses_cumulative_counts(self):
        model = WheelOdometry(encoder_counts_per_rev=100, noise_std_m_per_sqrt_m=0., right_scale_bias=0.)
        model.update(0., 0., 0.)
        self.assertEqual(model.update(.02, .001, .001).x_m, 0.)
        sample = model.update(.04, 2*math.pi/100, 2*math.pi/100)
        self.assertAlmostEqual(sample.x_m, .15*2*math.pi/100)

    def test_noise_is_reproducible_and_stationary_robot_does_not_random_walk(self):
        first, second = WheelOdometry(seed=42), WheelOdometry(seed=42)
        for i in range(50):
            self.assertEqual(first.update(i*.02, i*.01, i*.01), second.update(i*.02, i*.01, i*.01))
        before = (first.x, first.y, first.theta)
        for i in range(50, 100):
            first.update(i*.02, .49, .49)
        self.assertEqual(before, (first.x, first.y, first.theta))

    def test_time_reset_is_rejected(self):
        model = ideal()
        model.update(10., 0., 0.)
        with self.assertRaises(ValueError):
            model.update(0., 0., 0.)


class EvaluationTests(unittest.TestCase):
    def test_yaw_interpolation_crosses_pi_without_false_full_turn(self):
        first = Pose(1., 0., 0., math.radians(179))
        second = Pose(1.1, 2., 0., math.radians(-179))
        truth = interpolate_truth(first, second, 1.05)
        self.assertAlmostEqual(truth.x_m, 1.)
        self.assertAlmostEqual(abs(truth.theta_rad), math.pi)
        error = pose_error(Pose(1.05, 1., 1., math.radians(-179)), truth)
        self.assertAlmostEqual(error['position_error_m'], 1.)
        self.assertAlmostEqual(error['heading_error_rad'], math.radians(1))

    def test_delayed_truth_waits_for_bracket(self):
        aligner = TimeAligner()
        self.assertEqual(aligner.add_estimate(Pose(1.05, .5, 0., 0.)), [])
        self.assertEqual(aligner.add_truth(Pose(1., 0., 0., 0.)), [])
        rows = aligner.add_truth(Pose(1.08, .8, 0., 0.))
        self.assertEqual(len(rows), 1)
        self.assertAlmostEqual(rows[0]['position_error_m'], 0.)

    def test_large_truth_gap_is_not_extrapolated(self):
        aligner = TimeAligner()
        aligner.add_truth(Pose(1., 0., 0., 0.))
        aligner.add_truth(Pose(2., 1., 0., 0.))
        self.assertEqual(aligner.add_estimate(Pose(1.5, .5, 0., 0.)), [])
        self.assertEqual(aligner.dropped, 1)
        self.assertAlmostEqual(wrap_angle(math.radians(359)), math.radians(-1))


if __name__ == '__main__':
    unittest.main()
