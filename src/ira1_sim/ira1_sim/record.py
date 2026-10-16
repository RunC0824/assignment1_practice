"""Record independent raw streams. No pose estimate is generated here."""
import csv
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry

class Recorder(Node):
    def __init__(self):
        super().__init__('ira1_recorder')
        self.declare_parameter('output_dir', str(Path(__file__).resolve().parents[3] / 'data'))
        folder = Path(self.get_parameter('output_dir').value)
        folder.mkdir(parents=True, exist_ok=True)
        # Exclusive creation protects previous measurements.
        self.wf = (folder/'wheel_states.csv').open('x', newline='')
        self.gf = (folder/'ground_truth.csv').open('x', newline='')
        self.w = csv.writer(self.wf); self.g = csv.writer(self.gf)
        self.w.writerow(['time_s','left_angle_rad','right_angle_rad'])
        self.g.writerow(['time_s','x_m','y_m','z_m','qx','qy','qz','qw'])
        self.counts = [0,0]
        self.create_subscription(JointState, '/wheel_states', self.wheels, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/ground_truth', self.truth, qos_profile_sensor_data)
        self.create_timer(2., self.status)
        self.get_logger().info(f'Recording raw streams to {folder}')

    @staticmethod
    def stamp(msg):
        return msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9

    def wheels(self, msg):
        try:
            l = msg.name.index('left_wheel_joint'); r = msg.name.index('right_wheel_joint')
            row = [self.stamp(msg),msg.position[l],msg.position[r]]
        except (ValueError, IndexError):
            return
        self.w.writerow(row); self.counts[0] += 1

    def truth(self, msg):
        p,q = msg.pose.pose.position,msg.pose.pose.orientation
        self.g.writerow([self.stamp(msg),p.x,p.y,p.z,q.x,q.y,q.z,q.w]); self.counts[1] += 1

    def status(self):
        self.wf.flush(); self.gf.flush()
        self.get_logger().info(f'Rows: wheels={self.counts[0]}, ground_truth={self.counts[1]}')

def main():
    rclpy.init(); node = Recorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.wf.close(); node.gf.close()
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
