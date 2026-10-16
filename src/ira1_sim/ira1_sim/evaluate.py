"""Independent evaluation node. Ground truth is never published as an estimate."""
import csv
import json
import math
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from .evaluation import Pose, TimeAligner, pose_error


class Evaluator(Node):
    def __init__(self):
        super().__init__('ira1_evaluation')
        self.declare_parameter('output_dir', '')
        self.declare_parameter('max_truth_gap_s', 0.1)
        name = self.get_parameter('output_dir').value
        if not name:
            raise ValueError('Set output_dir to a new experiment directory')
        self.folder = Path(name)
        self.folder.mkdir(parents=True, exist_ok=True)
        self.file = (self.folder/'errors.csv').open('x', newline='')
        columns = list(pose_error(Pose(0,0,0,0), Pose(0,0,0,0)))
        self.writer = csv.DictWriter(self.file, fieldnames=columns)
        self.writer.writeheader()
        self.aligner = TimeAligner(self.get_parameter('max_truth_gap_s').value)
        self.count = 0
        self.sum_position_squared = self.sum_heading_squared = 0.0
        self.max_position = self.max_heading = 0.0
        self.first_time = None
        self.last = None
        self.create_subscription(Odometry, '/ground_truth', self.truth, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/estimated_pose', self.estimate, 20)
        self.create_timer(2.0, self.status)
        self.get_logger().info('Evaluation ready; truth is used only for time-aligned errors')

    @staticmethod
    def pose(msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        return Pose(msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9, p.x, p.y, yaw)

    def truth(self, msg):
        self.record(self.aligner.add_truth(self.pose(msg)))

    def estimate(self, msg):
        self.record(self.aligner.add_estimate(self.pose(msg)))

    def record(self, rows):
        for row in rows:
            self.writer.writerow(row)
            self.file.flush()
            self.count += 1
            self.sum_position_squared += row['position_error_m']**2
            self.sum_heading_squared += row['heading_error_rad']**2
            self.max_position = max(self.max_position, row['position_error_m'])
            self.max_heading = max(self.max_heading, abs(row['heading_error_rad']))
            if self.first_time is None:
                self.first_time = row['time_s']
            self.last = row

    def save_summary(self):
        self.file.flush()
        summary = dict(samples=self.count, dropped_unalignable_samples=self.aligner.dropped,
            pending_samples=len(self.aligner.pending), first_time_s=self.first_time,
            last_time_s=self.last['time_s'] if self.last else None,
            position_rmse_m=math.sqrt(self.sum_position_squared/self.count) if self.count else None,
            heading_rmse_rad=math.sqrt(self.sum_heading_squared/self.count) if self.count else None,
            max_position_error_m=self.max_position, max_abs_heading_error_rad=self.max_heading,
            last_position_error_m=self.last['position_error_m'] if self.last else None,
            last_heading_error_rad=self.last['heading_error_rad'] if self.last else None,
            alignment='linear x/y and shortest-angle yaw interpolation of bracketing simulator truth; no extrapolation',
            scope='Stage 2 functional run; statistics cover all aligned samples, including stops')
        temporary = self.folder/'summary.json.tmp'
        temporary.write_text(json.dumps(summary, indent=2)+'\n')
        temporary.replace(self.folder/'summary.json')

    def status(self):
        self.save_summary()
        if self.last:
            self.get_logger().info(f'N={self.count}, position error={self.last["position_error_m"]:.3f} m, '
                                   f'heading error={math.degrees(self.last["heading_error_rad"]):.2f} deg')


def main():
    rclpy.init()
    node = Evaluator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save_summary()
        node.file.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
