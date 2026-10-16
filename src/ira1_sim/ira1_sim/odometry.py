"""Lecture 2 midpoint wheel odometry, with explicit measurement assumptions.

This module has no ROS, simulator pose or ground-truth dependency.
Noise is applied to wheel travel, not to a simulator pose or a command.
"""
from dataclasses import dataclass
import math
import random


def wrap_angle(angle):
    """Signed angular difference in [-pi, pi)."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


@dataclass(frozen=True)
class Sample:
    time_s: float
    x_m: float
    y_m: float
    theta_rad: float
    dt_s: float = 0.0
    raw_left_m: float = 0.0
    raw_right_m: float = 0.0
    measured_left_m: float = 0.0
    measured_right_m: float = 0.0


class WheelOdometry:
    def __init__(self, wheel_radius_m=0.15, wheel_separation_m=0.625,
                 encoder_counts_per_rev=2048, update_period_s=0.02,
                 noise_std_m_per_sqrt_m=0.01,
                 left_scale_bias=0.0, right_scale_bias=0.01,
                 seed=42, initial_x_m=0.0, initial_y_m=0.0,
                 initial_theta_rad=0.0):
        if not all(math.isfinite(v) for v in [wheel_radius_m, wheel_separation_m,
                update_period_s, noise_std_m_per_sqrt_m, left_scale_bias,
                right_scale_bias, initial_x_m, initial_y_m, initial_theta_rad]):
            raise ValueError('All physical parameters must be finite')
        if wheel_radius_m <= 0 or wheel_separation_m <= 0 or update_period_s <= 0:
            raise ValueError('Geometry and update period must be positive')
        if encoder_counts_per_rev < 0 or noise_std_m_per_sqrt_m < 0:
            raise ValueError('Encoder resolution and noise scale cannot be negative')
        if min(left_scale_bias, right_scale_bias) <= -1.0:
            raise ValueError('Wheel scale factors must be positive')
        self.radius = wheel_radius_m
        self.separation = wheel_separation_m
        self.counts = encoder_counts_per_rev
        self.period = update_period_s
        self.noise = noise_std_m_per_sqrt_m
        self.biases = left_scale_bias, right_scale_bias
        self.rng = random.Random(seed)
        self.x, self.y, self.theta = initial_x_m, initial_y_m, initial_theta_rad
        self.previous = None
        self.last_seen_time = None

    def wheel_position(self, angle):
        # Zero counts selects ideal continuous encoders for diagnostic baselines.
        if self.counts:
            return round(angle * self.counts / (2.0 * math.pi)) * (2.0 * math.pi / self.counts)
        return angle

    def update(self, time_s, left_angle_rad, right_angle_rad):
        if not all(math.isfinite(v) for v in (time_s, left_angle_rad, right_angle_rad)):
            raise ValueError('Wheel sample must be finite')
        if self.last_seen_time is not None and time_s < self.last_seen_time - 1e-9:
            raise ValueError('Simulation time reset: restart the estimator for a new run')
        self.last_seen_time = time_s
        left, right = self.wheel_position(left_angle_rad), self.wheel_position(right_angle_rad)
        if self.previous is None:
            self.previous = time_s, left, right
            return Sample(time_s, self.x, self.y, self.theta)
        previous_time, previous_left, previous_right = self.previous
        dt = time_s - previous_time
        if dt + 1e-9 < self.period:
            return None
        raw = self.radius * (left - previous_left), self.radius * (right - previous_right)
        measured = []
        for displacement, bias in zip(raw, self.biases):
            sigma = self.noise * math.sqrt(abs(displacement))
            epsilon = self.rng.gauss(0.0, sigma) if sigma else 0.0
            measured.append((1.0 + bias) * displacement + epsilon)
        dl, dr = measured
        ds = (dr + dl) / 2.0
        dtheta = (dr - dl) / self.separation
        # Exactly the lecture's short-step midpoint approximation.
        self.x += ds * math.cos(self.theta + dtheta / 2.0)
        self.y += ds * math.sin(self.theta + dtheta / 2.0)
        self.theta = wrap_angle(self.theta + dtheta)
        self.previous = time_s, left, right
        return Sample(time_s, self.x, self.y, self.theta, dt, *raw, dl, dr)
