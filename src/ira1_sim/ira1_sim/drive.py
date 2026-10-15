"""Timed open-loop square motion; no localisation or ground-truth feedback."""
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class Drive(Node):
    def __init__(self):
        super().__init__('ira1_drive')
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.start = None
        self.done = False
        self.segment = None
        # Wait, then four 2 m straights and nominal 90 degree turns.
        self.plan = [(2., 0., 0.)] + [(8., .25, 0.), (1., 0., 0.),
                     (math.pi, 0., .5), (1., 0., 0.)] * 4
        self.timer = self.create_timer(.05, self.tick)

    def tick(self):
        now = self.get_clock().now().nanoseconds / 1e9
        if now <= 0:
            return
        if self.start is None:
            self.start = now
        elapsed = now - self.start
        command = Twist()
        for i, (duration, linear, angular) in enumerate(self.plan):
            if elapsed < duration:
                command.linear.x, command.angular.z = linear, angular
                if self.segment != i:
                    self.get_logger().info(f'Segment {i}: v={linear} m/s, w={angular} rad/s')
                    self.segment = i
                break
            elapsed -= duration
        else:
            if not self.done:
                self.get_logger().info('Square command sequence complete; holding zero velocity.')
                self.done = True
        self.pub.publish(command)

def main():
    rclpy.init()
    node = Drive()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.pub.publish(Twist())
            node.destroy_node()
            rclpy.shutdown()
