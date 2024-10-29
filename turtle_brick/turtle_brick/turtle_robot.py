"""
Homework 2 for ME 495: Embedded Systems in Robotics.

Moving a turtle robot in Rviz to catch a falling brick
"""
import math

from geometry_msgs.msg import PoseStamped, TransformStamped, Twist, Vector3

from nav_msgs.msg import Odometry

import numpy as np

import rclpy
import rclpy.parameter
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile

from sensor_msgs.msg import JointState

from tf2_ros import TransformBroadcaster
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

from turtlesim.msg import Pose





class Turtle_Robot_Node(Node):
    """

    Node that controls the movement of a turtle from the turtlesim package.

    PUBLISHERS:
        + JointState /joint_states - The joint states for each joint of the turtle robot
        + Odometry /odom - The position and orientation of the odom to base_link transform
        + Twist /turtlesim1/turtle1/cmd_vel - The velocity of the turtle moving between waypoints
    SUBSCRIBERS:
        + PoseStamped /goal_pose - The location of the 'goal' / the brick x, y location
        + Pose turtle1/pose - The current pose of the turtle
        +Tilt
    SERVICES:
        + None
    PARAMETERS:
        + None
    """

    def __init__(self):
        """Intialize the variables of the turtle bot node."""
        super().__init__('turtle_bot_node')
        self.frequency = 100.0
        self.interval = 1 / self.frequency
        qos = QoSProfile(depth=10, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.joint_state_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.wheel_radius = 0.04
        self.forward_velocity = 1.0
        self.max_velocity = 5.0
        self.min_velocity = 0.5
        self.angle = 0.0
        self.old_time = self.get_clock().now().nanoseconds
        self.old_time = self.old_time / 1e9 
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/turtlesim1/turtle1/cmd_vel', 10)
        self.goal_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        self.pose_subscription = self.create_subscription(Pose, '/turtlesim1/turtle1/pose', self.pose_callback, 10)
        self.delta_x = 0.0
        self.delta_y = 0.0
        self.delta_theta = 0.0
        self.pose = 0.0
        self._tmr = self.create_timer(self.interval, self.timer_callback)
        self.odomX = 5.54
        self.odomY = 5.54
        self.static_broadcaster = StaticTransformBroadcaster(self, qos)
        self.world_odom_tf = TransformStamped()
        self.world_odom_tf.header.stamp = self.get_clock().now().to_msg()
        self.world_odom_tf.header.frame_id = 'world'
        self.world_odom_tf.child_frame_id = 'odom'
        self.world_odom_tf.transform.translation.x = self.odomX
        self.world_odom_tf.transform.translation.y = self.odomY
        self.world_odom_tf.transform.translation.z = 0.0
        q = quaternion_from_euler(0, 0, 0)
        self.world_odom_tf.transform.rotation.x = q[0]
        self.world_odom_tf.transform.rotation.y = q[1]
        self.world_odom_tf.transform.rotation.z = q[2]
        self.world_odom_tf.transform.rotation.w = q[3]
        self.goal_x = None
        self.goal_y = None
        self.distance_to_goal = 0.0
        self.threshold = 0.1
        self.declare_parameter('max_velocity', value=3.0)
        self.max_velocity = self.get_parameter('max_velocity').get_parameter_value().double_value
        self.static_broadcaster.sendTransform(self.world_odom_tf)
        self.broadcaster = TransformBroadcaster(self, 10)

    def timer_callback(self):
        """Timer that publishes joint states, broadcasts non-static frames, calculates the movement to the goal pose, and publishes the odometry message."""
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = ['j1', 'j2', 'j3']
        joint_state_msg.position = [0.0, 0.0, 0.0]
        self.joint_state_pub.publish(joint_state_msg)

        odom_base_link = TransformStamped()
        odom_base_link.header.frame_id = 'odom'
        odom_base_link.child_frame_id = 'base_link'
        odom_base_link.transform.translation.x = self.delta_x - self.odomX
        odom_base_link.transform.translation.y = self.delta_y - self.odomY
        odom_base_link.transform.translation.z = 0.0
        q = quaternion_from_euler(0, 0, self.delta_theta)
        odom_base_link.transform.rotation.x = q[0]
        odom_base_link.transform.rotation.y = q[1]
        odom_base_link.transform.rotation.z = q[2]
        odom_base_link.transform.rotation.w = q[3]
        self.broadcaster.sendTransform(odom_base_link)

        if self.goal_x is not None and self.goal_y is not None:
            self.distance_to_goal = self.get_distance_from_goal()

            if self.distance_to_goal > self.threshold:
                self.angle = math.atan2(self.goal_y - self.delta_y, self.goal_x - self.delta_x)
                self.distance_to_goal = math.sqrt((self.goal_y - self.delta_y)**2 + (self.goal_x - self.delta_x)**2)

                if self.angle > np.pi:
                    self.angle = self.angle - 2*np.pi
                elif self.angle < -np.pi:
                    self.angle = self.angle + 2*np.pi

                cmd_vel_msg = self.move_turtle(self.max_velocity * math.cos(self.angle), self.max_velocity * math.sin(self.angle))
                self.cmd_vel_pub.publish(cmd_vel_msg)

            else:
                cmd_vel_msg = self.move_turtle(0, 0)
                self.cmd_vel_pub.publish(cmd_vel_msg)
        else:
            cmd_vel_msg = self.move_turtle(0, 0)
            self.cmd_vel_pub.publish(cmd_vel_msg)

        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        odom_msg.pose.pose.position.x = self.delta_x
        odom_msg.pose.pose.position.y = self.delta_y
        odom_msg.pose.pose.orientation.x = q[0]
        odom_msg.pose.pose.orientation.y = q[1]
        odom_msg.pose.pose.orientation.z = q[2]
        odom_msg.pose.pose.orientation.w = q[3]
        odom_msg.twist.twist = cmd_vel_msg
        self.odom_pub.publish(odom_msg)

    def move_turtle(self, xdot: float, ydot: float) -> Twist:
        """Create a Twist message to publish on the cmd_vel topic.

        Args:
            xdot (float): linear translation in the x direction
            ydot (float): linear translation in the y direction

        Returns:
            Twist: Object that contains linear and angular components to be published on the cmd_vel topic
        """
        return Twist(linear=Vector3(x=float(xdot), y=float(ydot), z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.0))

    def pose_callback(self, msg: Pose):
        """Get the current pose of the turtlesim turtle.

        Args:
            msg (Pose): The x, y, and theta position and orientation of the turtle
        """
        self.delta_x = msg.x
        self.delta_y = msg.y
        self.delta_theta = msg.theta

    def goal_callback(self, msg: PoseStamped):
        """Locate the goal pose.

        Args:
            msg (PoseStamped): The x and y position of the goal
        """
        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y

    def get_distance_from_goal(self):
        """Calculate distance from the robot to the goal."""
        dist = math.sqrt((self.goal_y - self.delta_y)**2 + (self.goal_x - self.delta_x)**2)
        return dist


def quaternion_from_euler(ai: float, aj: float, ak: float) -> list:
    """Return the quaternion based on x, y, and theta.

    Args:
        ai (float): x 
        aj (float): y
        ak (float): theta

    Returns:
        list: Quaternion list with 4 elements (x, y, z, w)
    """
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss

    return q


def main(args=None):
    """Entrypoint for the turtle_robot ROS2 node."""
    rclpy.init(args=args)
    node = Turtle_Robot_Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    import sys
    main(sys.argv)