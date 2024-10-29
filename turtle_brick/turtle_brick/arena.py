"""Create the arena for the turtle robot."""
import math

from geometry_msgs.msg import Point, TransformStamped

import rclpy
import rclpy.parameter
from rclpy.node import Node
import rclpy.time
from rclpy.qos import QoSDurabilityPolicy, QoSProfile

from std_srvs.srv import Empty

from tf2_ros import TransformBroadcaster

from turtle_brick.physics import World

from turtle_brick_interfaces.srv import Place

from turtlesim.msg import Pose

from visualization_msgs.msg import Marker, MarkerArray


class Arena_Node(Node):
    """
    Node that build the arena elements including walls and brick.

    PUBLISHERS:
        + MarkerArray /visualization_marker_array - Walls markers for the arena
        + Marker /visualization_marker - Brick marker for the arena 
        + Point /brick_location_topic - The current position of the brick

    SUBSCRIBERS:
        + None

    SERVICES:
        + Place (Point) - Place the brick at a given location
        + Empty (Empty) - Drops the brick

    PARAMETERS:
        + None
    """

    def __init__(self):
        """Initialize the arena."""
        super().__init__('arena_node')
        self.frequency = 250
        self.interval = 1/self.frequency
        self._tmr = self.create_timer(self.interval, self.timer_callback)
        qos = QoSProfile(depth=10, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.marker_array_pub = self.create_publisher(MarkerArray, '/visualization_marker_array', qos)
        self.brick_pub = self.create_publisher(Marker, '/visualization_marker', qos)
        self.brick_location_pub = self.create_publisher(Point, '/brick_location_topic', qos)
        self.pose_subscription = self.create_subscription(Pose, '/turtlesim1/turtle1/pose', self.pose_callback, 10)
        self.location = None
        self.m1 = None
        self.m2 = None
        self.m3 = None
        self.m4 = None
        self.delta_x = 0.0
        self.delta_y = 0.0
        self.broadcaster = TransformBroadcaster(self, qos)
        self.place_brick_service = self.create_service(Place, 'place_brick', self.place_brick)
        self.drop_brick_service = self.create_service(Empty, 'drop_brick', self.drop_brick)
        self.brick_location = None
        self.brick_location = (0.0, 0.0, 0.0)
        self.gravity = 9.8
        self.brick_radius = 0.25
        self.world = World(self.brick_location, self.gravity, self.brick_radius, self.interval)
        self.drop_state = False
        self.place_state = False
        self.caught_state = False
        self.can_catch = True
        self.declare_parameter('platform_height', value=2.0)
        self.declare_parameter('gravity_accel', value=9.8)
        self.declare_parameter('max_velocity', value=3.0)
        self.platform_height = self.get_parameter('platform_height').get_parameter_value().double_value
        self.gravity_accel = self.get_parameter('gravity_accel').get_parameter_value().double_value
        self.max_velocity = self.get_parameter('max_velocity').get_parameter_value().double_value
        self.platform_length = self.platform_height / 10

    def timer_callback(self):
        """Broadcast the world to brick transform, places the brick marker, and drops the brick at a fixed frequency."""
        self.place_walls()
        if self.place_state:
            world_brick_tf = TransformStamped()
            world_brick_tf.header.frame_id = 'world'
            world_brick_tf.child_frame_id = 'brick'
            world_brick_tf.transform.translation.x = self.world.brick[0]
            world_brick_tf.transform.translation.y = self.world.brick[1]
            world_brick_tf.transform.translation.z = self.world.brick[2]
            self.broadcaster.sendTransform(world_brick_tf)
            self.brick_marker = Marker()
            self.brick_marker.header.frame_id = 'world'
            self.brick_marker.header.stamp = self.get_clock().now().to_msg()
            self.brick_marker.id = 5
            self.brick_marker.type = Marker.CUBE
            self.brick_marker.action = Marker.ADD
            self.brick_marker.scale.x = 0.5
            self.brick_marker.scale.y = 0.5
            self.brick_marker.scale.z = 0.5
            self.brick_marker.pose.position.x = self.world.brick[0]
            self.brick_marker.pose.position.y = self.world.brick[1]
            self.brick_marker.pose.position.z = self.world.brick[2]
            self.brick_marker.color.r = 0.0
            self.brick_marker.color.g = 1.0
            self.brick_marker.color.b = 1.0
            self.brick_marker.color.a = 1.0
            self.brick_pub.publish(self.brick_marker)
            if self.drop_state:
                self.location = Point()
                self.location.x = self.world.brick[0]
                self.location.y = self.world.brick[1]
                self.location.z = self.world.brick[2]
                self.brick_location_pub.publish(self.location)
                self.world.drop()

            if self.can_catch:
                if self.world.brick[2] <= (self.platform_height + (self.platform_height / (self.platform_height*2)) - self.platform_length):
                    self.drop_state = False
                    self.caught_state = True
                    self.place_state = False
            else:
                self.text_marker = Marker()
                self.text_marker.header.frame_id = 'world'
                self.text_marker.header.stamp = self.get_clock().now().to_msg()
                self.text_marker.id = 5
                self.text_marker.type = Marker.TEXT_VIEW_FACING
                self.text_marker.action = Marker.ADD
                self.text_marker.scale.x = 0.0
                self.text_marker.scale.y = 0.0
                self.text_marker.scale.z = 1.0
                self.text_marker.text = 'Unreachable'
                self.text_marker.pose.position.x = 0.0
                self.text_marker.pose.position.y = 0.0
                self.text_marker.pose.position.z = 0.0
                self.text_marker.color.r = 1.0
                self.text_marker.color.g = 0.0
                self.text_marker.color.b = 0.0
                self.text_marker.color.a = 1.0
                self.brick_pub.publish(self.text_marker)
                

        if self.caught_state:
            world_brick_tf = TransformStamped()
            world_brick_tf.header.frame_id = 'world'
            world_brick_tf.child_frame_id = 'brick'
            world_brick_tf.transform.translation.x = self.delta_x
            world_brick_tf.transform.translation.y = self.delta_y
            world_brick_tf.transform.translation.z = self.world.brick[2]
            self.broadcaster.sendTransform(world_brick_tf)
            self.brick_marker = Marker()
            self.brick_marker.header.frame_id = 'world'
            self.brick_marker.header.stamp = self.get_clock().now().to_msg()
            self.brick_marker.id = 5
            self.brick_marker.type = Marker.CUBE
            self.brick_marker.action = Marker.ADD
            self.brick_marker.scale.x = 0.5
            self.brick_marker.scale.y = 0.5
            self.brick_marker.scale.z = 0.5
            self.brick_marker.pose.position.x = self.delta_x
            self.brick_marker.pose.position.y = self.delta_y
            self.brick_marker.pose.position.z = self.world.brick[2]
            self.brick_marker.color.r = 0.0
            self.brick_marker.color.g = 1.0
            self.brick_marker.color.b = 1.0
            self.brick_marker.color.a = 1.0
            self.brick_pub.publish(self.brick_marker)

    def place_brick(self, request: Point, response: Empty) -> Empty:
        """
        Service that places the brick based on the location specified.

        Args:
            request (Point): x, y, z coordinates of the brick
            response (Empty): 

        Returns:
            Empty:
        """
        self.place_state = True
        self.world.brick = (request.brick_location.x, request.brick_location.y, request.brick_location.z)

        return response

    def drop_brick(self, request: Empty, response: Empty) -> Empty:
        """Service that toggle the drop_brick boolean to drop the brick."""
        self.drop_state = True
        z_displacement = self.world.brick[2] - self.platform_height
        brick_time = math.sqrt((2 * abs(z_displacement)) / self.gravity_accel)

        robot_distance = math.sqrt((self.world.brick[1] - self.delta_y)**2 + (self.world.brick[0] - self.delta_x)**2)
        robot_time = robot_distance / self.max_velocity

        if brick_time < robot_time:
            self.can_catch = False
        else:
            self.can_catch = True
        return response

    def pose_callback(self, msg: Pose):
        """Get the current pose of the turtlesim turtle.

        Args:
            msg (Pose): The x, y, and theta position and orientation of the turtle
        """
        self.delta_x = msg.x
        self.delta_y = msg.y

    def place_walls(self):
        """Publish the walls as a marker array in Rviz."""
        self.m1 = Marker()
        self.m1.header.frame_id = 'world'
        self.m1.header.stamp = self.get_clock().now().to_msg()
        self.m1.id = 1
        self.m1.type = Marker.CUBE
        self.m1.action = Marker.ADD
        self.m1.scale.x = 0.5
        self.m1.scale.y = 1.0
        self.m1.scale.z = 10.0
        self.m1.pose.position.x = 0.5
        self.m1.pose.position.y = 5.5
        self.m1.pose.position.z = 0.5
        self.m1.pose.orientation.x = .707
        self.m1.pose.orientation.y = 0.0
        self.m1.pose.orientation.z = 0.0
        self.m1.pose.orientation.w = .707
        self.m1.color.r = 0.0
        self.m1.color.g = 0.0
        self.m1.color.b = 1.0
        self.m1.color.a = 1.0

        self.m2 = Marker()
        self.m2.header.frame_id = 'world'
        self.m2.header.stamp = self.get_clock().now().to_msg()
        self.m2.id = 2
        self.m2.type = Marker.CUBE
        self.m2.action = Marker.ADD
        self.m2.scale.x = 0.5
        self.m2.scale.y = 1.0
        self.m2.scale.z = 10.0
        self.m2.pose.position.x = 10.5
        self.m2.pose.position.y = 5.5
        self.m2.pose.position.z = 0.5
        self.m2.pose.orientation.x = .707
        self.m2.pose.orientation.y = 0.0
        self.m2.pose.orientation.z = 0.0
        self.m2.pose.orientation.w = .707
        self.m2.color.r = 0.0
        self.m2.color.g = 0.0
        self.m2.color.b = 1.0
        self.m2.color.a = 1.0

        self.m3 = Marker()
        self.m3.header.frame_id = 'world'
        self.m3.header.stamp = self.get_clock().now().to_msg()
        self.m3.id = 3
        self.m3.type = Marker.CUBE
        self.m3.action = Marker.ADD
        self.m3.scale.x = 0.5
        self.m3.scale.y = 1.0
        self.m3.scale.z = 10.0
        self.m3.pose.position.x = 5.5
        self.m3.pose.position.y = 0.5
        self.m2.pose.position.z = 0.5
        self.m3.pose.orientation.x = .707
        self.m3.pose.orientation.y = 0.0
        self.m3.pose.orientation.z = .707
        self.m3.pose.orientation.w = 0.0
        self.m3.color.r = 0.0
        self.m3.color.g = 1.0
        self.m3.color.b = 0.0
        self.m3.color.a = 1.0

        self.m4 = Marker()
        self.m4.header.frame_id = 'world'
        self.m4.header.stamp = self.get_clock().now().to_msg()
        self.m4.id = 4
        self.m4.type = Marker.CUBE
        self.m4.action = Marker.ADD
        self.m4.scale.x = 0.5
        self.m4.scale.y = 1.0
        self.m4.scale.z = 10.0
        self.m4.pose.position.x = 5.5
        self.m4.pose.position.y = 10.5
        self.m4.pose.position.z = 0.5
        self.m4.pose.orientation.x = .707
        self.m4.pose.orientation.y = 0.0
        self.m4.pose.orientation.z = .707
        self.m4.pose.orientation.w = 0.0
        self.m4.color.r = 0.0
        self.m4.color.g = 1.0
        self.m4.color.b = 0.0
        self.m4.color.a = 1.0

        self.marker_array = MarkerArray()
        self.marker_array.markers = [self.m1, self.m2, self.m3, self.m4]
        self.marker_array_pub.publish(self.marker_array)


def main(args=None):
    """Entrypoint for the Arena ROS2 node."""
    rclpy.init(args=args)
    node = Arena_Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    import sys
    main(sys.argv)
