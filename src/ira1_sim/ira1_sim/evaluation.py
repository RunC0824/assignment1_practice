"""Time alignment and pose errors for evaluation only; never fed to odometry."""
from collections import deque
from dataclasses import dataclass
from .odometry import wrap_angle
import math


@dataclass(frozen=True)
class Pose:
    time_s: float
    x_m: float
    y_m: float
    theta_rad: float


def interpolate_truth(first, second, time_s):
    if not first.time_s <= time_s <= second.time_s:
        raise ValueError('Truth interpolation requires a bracketing interval')
    fraction = ((time_s-first.time_s)/(second.time_s-first.time_s)
                if second.time_s > first.time_s else 0.0)
    return Pose(time_s, first.x_m+fraction*(second.x_m-first.x_m),
                first.y_m+fraction*(second.y_m-first.y_m),
                wrap_angle(first.theta_rad+fraction*wrap_angle(second.theta_rad-first.theta_rad)))


def pose_error(estimated, truth):
    ex, ey = estimated.x_m-truth.x_m, estimated.y_m-truth.y_m
    return dict(time_s=estimated.time_s, estimated_x_m=estimated.x_m,
                estimated_y_m=estimated.y_m, estimated_theta_rad=estimated.theta_rad,
                truth_x_m=truth.x_m, truth_y_m=truth.y_m, truth_theta_rad=truth.theta_rad,
                error_x_m=ex, error_y_m=ey, position_error_m=math.hypot(ex, ey),
                heading_error_rad=wrap_angle(estimated.theta_rad-truth.theta_rad))


class TimeAligner:
    def __init__(self, max_truth_gap_s=0.1):
        self.truth = deque(maxlen=5000)
        self.pending = deque()
        self.max_gap = max_truth_gap_s
        self.dropped = 0

    def add_truth(self, pose):
        if self.truth and pose.time_s < self.truth[-1].time_s:
            raise ValueError('Ground-truth time reset: restart the experiment')
        self.truth.append(pose)
        return self.drain()

    def add_estimate(self, pose):
        if len(self.pending) >= 500:
            self.pending.popleft()
            self.dropped += 1
        self.pending.append(pose)
        return self.drain()

    def drain(self):
        result = []
        while self.pending and self.truth:
            estimate = self.pending[0]
            if estimate.time_s < self.truth[0].time_s:
                self.pending.popleft()
                self.dropped += 1
                continue
            while len(self.truth) >= 2 and self.truth[1].time_s <= estimate.time_s:
                self.truth.popleft()
            if abs(estimate.time_s-self.truth[0].time_s) < 1e-9:
                truth = self.truth[0]
            elif len(self.truth) < 2:
                break  # Wait for a future truth sample; never extrapolate.
            elif self.truth[1].time_s-self.truth[0].time_s > self.max_gap:
                self.pending.popleft()
                self.dropped += 1
                continue
            else:
                truth = interpolate_truth(self.truth[0], self.truth[1], estimate.time_s)
            self.pending.popleft()
            result.append(pose_error(estimate, truth))
        return result
