"""ROS2 Node that causes the turtle robot to catch the falling brick."""
import math

from builtin_interfaces.msg import Duration

from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped

import rclpy
import rclpy.parameter
import rclpy.time
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile

from turtlesim.msg import Pose

from visualization_msgs.msg import Marker

class Arena_Node(Node):
    """Catch the falling brick."""

    def __init__(self):
        """Initialize the variables needed to catch the falling brick."""
        super().__init__('catcher_node')
        self.frequency = 250
        self.interval = 1/self.frequency
        self._tmr = self.create_timer(self.interval, self.timer_callback)

        qos = QoSProfile(depth=10, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

        self.brick_sub = self.create_subscription(Point, '/brick_location_topic', callback=self.brick_state_callback, qos_profile=qos)
        self.pose_subscription = self.create_subscription(Pose, '/turtlesim1/turtle1/pose', self.pose_callback, 10)
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', qos)
        self.text_pub = self.create_publisher(Marker, '/visualization_marker', qos)
        self.goal = None
        self.origin_goal = None
        self.drop_state = False

        self.declare_parameter("platform_height", value=2.0)
        self.declare_parameter("max_velocity", value=3.0)
        self.declare_parameter("gravity_accel", value=9.8)
        self.platform_height = self.get_parameter("platform_height").get_parameter_value().double_value
        self.max_velocity = self.get_parameter("max_velocity").get_parameter_value().double_value
        self.gravity_accel = self.get_parameter("gravity_accel").get_parameter_value().double_value
        self.platform_length = self.platform_height / 10

        self.brick_x = 0.0
        self.brick_y = 0.0
        self.brick_z = 0.0
        self.current_x = 0.0
        self.current_y = 0.0
        self.can_catch = False
        self.run_first = True

    def timer_callback(self):
        """Publish goal frame at a fixed frequency."""
        if self.can_catch:
            self.goal = PoseStamped()
            self.goal.header.stamp = self.get_clock().now().to_msg()
            self.goal.header.frame_id = "world"
            self.goal.pose.position.x = self.brick_x
            self.goal.pose.position.y = self.brick_y
            self.goal_pub.publish(self.goal)

        if self.brick_z <= (self.platform_height + (self.platform_height / (self.platform_height*2)) - self.platform_length):
            self.origin_goal = PoseStamped()
            self.origin_goal.header.stamp = self.get_clock().now().to_msg()
            self.origin_goal.header.frame_id = "world"
            self.origin_goal.pose.position.x = 5.45
            self.origin_goal.pose.position.y = 5.45
            self.goal_pub.publish(self.origin_goal)
            self.can_catch = False

    def brick_state_callback(self, msg):
        """Check if the robot can reach the brick in time."""
        self.brick_x = msg.x
        self.brick_y = msg.y
        self.brick_z = msg.z
        if msg is not None:
            self.drop_state = True
            z_displacement = self.brick_z - self.platform_height
            if self.run_first:
                self.is_catch_valid(z_displacement)
                self.run_first = False

    def pose_callback(self, msg):
        """Get the current pose of the robot."""
        self.current_x = msg.x
        self.current_y = msg.y

    def is_catch_valid(self, h):
        """Check if the robot is able to get to the brick in time."""
        brick_time = math.sqrt((2 * h) / self.gravity_accel)

        robot_distance = self.get_distance_from_goal()
        robot_time = robot_distance / self.max_velocity

        if brick_time < robot_time:
            self.get_logger().error("UNREACHABLE")
            self.can_catch = False
            self.text_marker = Marker()
            self.text_marker.header.frame_id = 'world'
            self.text_marker.header.stamp = self.get_clock().now().to_msg()
            self.text_marker.id = 10
            self.text_marker.type = Marker.TEXT_VIEW_FACING
            self.text_marker.action = Marker.ADD
            self.text_marker.scale.x = 0.0
            self.text_marker.scale.y = 0.0
            self.text_marker.scale.z = 1.0
            self.text_marker.text = "UNREACHABLE"
            self.text_marker.pose.position.x = 5.5
            self.text_marker.pose.position.y = 5.5
            self.text_marker.pose.position.z = 1.0
            self.text_marker.color.r = 1.0
            self.text_marker.color.g = 0.0
            self.text_marker.color.b = 1.0
            self.text_marker.color.a = 1.0
            self.text_marker.lifetime = Duration(seconds=3)
            self.text_pub.publish(self.text_marker)
        else:
            self.can_catch = True

    def get_distance_from_goal(self):
        """Calculate the distance from the robot to the goal."""
        dist = math.sqrt((self.brick_y - self.current_y)**2 + (self.brick_x - self.current_x)**2)
        return dist


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
