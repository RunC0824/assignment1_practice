"""ROS wheel-only estimator. It never subscribes to simulator truth."""
import csv
from dataclasses import asdict, fields
import json
import math
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from .odometry import WheelOdometry, Sample


DEFAULTS = dict(wheel_radius_m=0.15, wheel_separation_m=0.625,
                encoder_counts_per_rev=2048, update_period_s=0.02,
                noise_std_m_per_sqrt_m=0.01,
                left_scale_bias=0.0, right_scale_bias=0.01,
                seed=42, initial_x_m=0.0, initial_y_m=0.0, initial_theta_rad=0.0)


class Estimator(Node):
    def __init__(self):
        super().__init__('ira1_odometry')
        self.declare_parameter('output_dir', '')
        for name, value in DEFAULTS.items():
            self.declare_parameter(name, value)
        config = {name: self.get_parameter(name).value for name in DEFAULTS}
        self.model = WheelOdometry(**config)
        folder = Path(self.get_parameter('output_dir').value)
        if not self.get_parameter('output_dir').value:
            raise ValueError('Set output_dir to a new experiment directory')
        folder.mkdir(parents=True, exist_ok=True)
        self.file = (folder/'estimated_pose.csv').open('x', newline='')
        self.writer = csv.DictWriter(self.file, fieldnames=[field.name for field in fields(Sample)])
        self.writer.writeheader()
        metadata = dict(parameters=config, method='Lecture 2 midpoint wheel odometry',
            measurement_model='d_measured=(1+bias)*d_quantised+Normal(0,k*sqrt(abs(d_quantised)))',
            noise_units='k in m/sqrt(m)',
            initial_pose_source='Known experiment configuration; never read from ground truth',
            input_topic='/wheel_states', output_topic='/estimated_pose',
            ground_truth_used_for_estimation=False)
        (folder/'estimator_config.json').write_text(json.dumps(metadata, indent=2)+'\n')
        self.pub = self.create_publisher(Odometry, '/estimated_pose', 20)
        self.create_subscription(JointState, '/wheel_states', self.wheels, qos_profile_sensor_data)
        self.create_timer(1.0, self.file.flush)
        self.get_logger().info(f'Wheel-only estimator ready; output={folder}')

    def wheels(self, msg):
        try:
            left = msg.position[msg.name.index('left_wheel_joint')]
            right = msg.position[msg.name.index('right_wheel_joint')]
        except (ValueError, IndexError):
            self.get_logger().error('Wheel state does not contain both wheel positions', once=True)
            return
        time_s = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        try:
            sample = self.model.update(time_s, left, right)
        except ValueError as error:
            self.get_logger().error(str(error), once=True)
            raise
        if sample is None:
            return
        self.writer.writerow(asdict(sample))
        self.file.flush()  # Save the evidence before publishing the corresponding pose.
        out = Odometry()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = 'world'
        out.child_frame_id = 'ira1_robot_estimated'
        out.pose.pose.position.x, out.pose.pose.position.y = sample.x_m, sample.y_m
        out.pose.pose.orientation.z = math.sin(sample.theta_rad / 2.0)
        out.pose.pose.orientation.w = math.cos(sample.theta_rad / 2.0)
        if sample.dt_s:
            out.twist.twist.linear.x = (sample.measured_left_m + sample.measured_right_m) / (2.0 * sample.dt_s)
            out.twist.twist.angular.z = (sample.measured_right_m - sample.measured_left_m) / (self.model.separation * sample.dt_s)
        # Default ROS covariance fields are not an uncertainty estimate. We do not
        # propagate covariance or use a filter; uncertainty assumptions are in JSON.
        self.pub.publish(out)


def main():
    rclpy.init()
    node = Estimator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.file.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
