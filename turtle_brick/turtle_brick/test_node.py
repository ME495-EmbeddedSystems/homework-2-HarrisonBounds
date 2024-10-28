"""ROS2 Node that causes the turtle robot to catch the falling brick"""
"""ROS2 Node that simulates the environment"""
import rclpy
from rclpy.node import Node
import rclpy.parameter
import rclpy.time
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from geometry_msgs.msg import Point
from turtlesim.msg import Pose
import math
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState

class Arena_Node(Node):
    def __init__(self):
        super().__init__('test_node')
        self.joint_state_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.frequency = 100
        self.interval = 1 / self.frequency
        self._tmr = self.create_timer(self.interval, self.timer_callback)
    
    def timer_callback(self):
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = ['j1', 'j2', 'j3']
        js.position = [1.0, 1.0, 1.0]
        self.joint_state_pub.publish(js)
        
    
def main(args=None):
    """Entrypoint for the mynode ROS node."""
    rclpy.init(args=args)
    node = Arena_Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    import sys
    main(sys.argv)
    