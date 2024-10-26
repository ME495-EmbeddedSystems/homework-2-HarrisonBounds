"""ROS2 Node that causes the turtle robot to catch the falling brick"""
"""ROS2 Node that simulates the environment"""
import rclpy
from rclpy.node import Node
import rclpy.parameter
import rclpy.time
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped, Point
from turtle_brick_interfaces.srv import Place
from turtle_brick.physics import World
from std_srvs.srv import Empty
from turtlesim.msg import Pose
import math

class Arena_Node(Node):
    def __init__(self):
        super().__init__('catcher_node')
        self.frequency = 250
        self.interval = 1/self.frequency
        self._tmr = self.create_timer(self.interval, self.timer_callback)
        
        qos = QoSProfile(depth=10, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        
        self.brick_sub = self.create_subscription(Point, "/brick_location_topic", callback=self.brick_state_callback, qos_profile=qos)
        self.pose_subscription = self.create_subscription(Pose, '/turtlesim1/turtle1/pose', self.pose_callback, 10)
        self.drop_state = False
        
        self.declare_parameter("platform_height", value=2.0)
        self.declare_parameter("max_velocity", value=3.0)
        self.declare_parameter("gravity_accel", value=9.8)
        self.platform_height = self.get_parameter("platform_height").get_parameter_value().double_value
        self.max_velocity = self.get_parameter("max_velocity").get_parameter_value().double_value
        self.gravity_accel = self.get_parameter("gravity_accel").get_parameter_value().double_value
        
        self.brick_x = 0.0
        self.brick_y = 0.0
        self.brick_z = 0.0
        self.current_x = 0.0
        self.current_y = 0.0
        
    
    def timer_callback(self):
        #Calculate if the robot can catch the brick or not
        if self.drop_state:
            z_displacement = self.brick_z - self.platform_height
            self.is_catch_valid(z_displacement)
            pass
            
    def brick_state_callback(self, msg):
        self.brick_x = msg.x
        self.brick_y = msg.y
        self.brick_z = msg.z
        if msg is not None:
            self.drop_state = True
            
    def pose_callback(self, msg):
        self.current_x = msg.x
        self.current_y = msg.y
            
    def is_catch_valid(self, h):
        brick_time = math.sqrt((2 * h) / self.gravity_accel)
        
        robot_distance = self.get_distance_from_goal()
        robot_time = robot_distance / self.max_velocity
        
        if brick_time > robot_time:
            self.get_logger().error("UNREACHABLE")
        else:
            
        
    def get_distance_from_goal(self):
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
    