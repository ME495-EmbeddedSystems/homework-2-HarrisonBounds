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
from std_msgs.msg import Empty


class Arena_Node(Node):
    def __init__(self):
        super().__init__('arena_node')
        self.frequency = 250
        self.interval = 1/self.frequency
        self._tmr = self.create_timer(self.interval, self.timer_callback)
        
        qos = QoSProfile(depth=10, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
    
        self.marker_array_pub = self.create_publisher(MarkerArray, '/visualization_marker_array', qos)
        self.brick_pub = self.create_publisher(Marker, "/visualization_marker", qos)
        
        self.m1 = None
        self.m2 = None
        self.m3 = None
        self.m4 = None
        
        self.broadcaster = TransformBroadcaster(self, qos)
        
        self.place_brick_service = self.create_service(Place, 'place_brick', self.place_brick)
        self.drop_brick_service = self.create_service(Empty, 'drop_brick', self.drop_brick)
        
        self.brick_location = None
        
        #Set the physics
        self.brick_location = (0.0, 0.0, 0.0)
        self.gravity = 9.8
        self.brick_radius = 0.25
        self.dt = 1/250
        
        self.world = World(self.brick_location, self.gravity, self.brick_radius, self.dt)
    
        
    def timer_callback(self):
        self.place_walls()
        
        
        
    def place_brick(self, request, response):
        
        world_brick_tf = TransformStamped()
        world_brick_tf.header.frame_id = 'world'
        world_brick_tf.child_frame_id = 'brick'
        world_brick_tf.transform.translation.x = request.brick_location.x
        world_brick_tf.transform.translation.y = request.brick_location.y
        world_brick_tf.transform.translation.z = request.brick_location.y
        self.broadcaster.sendTransform(world_brick_tf)
        
        self.brick = Marker()
        self.brick.header.frame_id = 'world'
        self.brick.header.stamp = self.get_clock().now().to_msg()
        self.brick.id = 5
        self.brick.type = Marker.CUBE
        self.brick.action = Marker.ADD
        self.brick.scale.x = 0.5
        self.brick.scale.y = 0.5
        self.brick.scale.z = 0.5
        self.brick.pose.position.x = request.brick_location.x
        self.brick.pose.position.y = request.brick_location.y
        self.brick.pose.position.z = request.brick_location.z
        self.brick.color.r = 0.0
        self.brick.color.g = 1.0
        self.brick.color.b = 1.0
        self.brick.color.a = 1.0
        
        self.brick_pub.publish(self.brick)
        
        #Set the brick location 
        self.world.brick = (request.brick_location.x, request.brick_location.y, request.brick_location.z)
        
        return response
    
    def drop_brick(self, request, response):
        self.world.drop()
        
        return response
        
    def place_walls(self):
        #Wall 1
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
        
        
        # #Wall 2
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
        
        #Wall 3
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
        
        #Wall 4
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
    """Entrypoint for the mynode ROS node."""
    rclpy.init(args=args)
    node = Arena_Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    import sys
    main(sys.argv)
    