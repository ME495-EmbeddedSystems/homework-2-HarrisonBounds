"""ROS2 Node that simulates the environment"""
import rclpy
from rclpy.node import Node
import rclpy.parameter
import rclpy.time
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray


class Arena_Node(Node):
    def __init__(self):
        super().__init__('arena_node')
        self.frequency = 250
        self.interval = 1/self.frequency
        self._tmr = self.create_timer(self.interval, self.timer_callback)
        
        markerQoS = QoSProfile(depth=10, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.marker_array_pub = self.create_publisher(MarkerArray, '/visualization_marker_array', markerQoS)
        
        self.m1 = None
        self.m2 = None
        
    def timer_callback(self):
        #Wall 1
        self.m1 = Marker()
        self.m1.header.frame_id = 'odom'
        self.m1.header.stamp = self.get_clock().now().to_msg()
        self.m1.id = 1
        self.m1.type = Marker.CUBE
        self.m1.action = Marker.ADD
        self.m1.scale.x = 0.5
        self.m1.scale.y = 1.0
        self.m1.scale.z = 10.0
        self.m1.pose.position.x = 5.0
        self.m1.pose.position.y = 0.0
        self.m1.pose.position.z = 0.5
        self.m1.pose.orientation.x = .707
        self.m1.pose.orientation.y = 0.0
        self.m1.pose.orientation.z = 0.0
        self.m1.pose.orientation.w = .707
        self.m1.color.r = 0.0
        self.m1.color.g = 0.0
        self.m1.color.b = 1.0
        self.m1.color.a = 1.0
        
        
        #Wall 2
        self.m2 = Marker()
        self.m2.header.frame_id = 'odom'
        self.m2.header.stamp = self.get_clock().now().to_msg()
        self.m2.id = 1
        self.m2.type = Marker.CUBE
        self.m2.action = Marker.ADD
        self.m2.scale.x = 0.5
        self.m2.scale.y = 1.0
        self.m2.scale.z = 10.0
        self.m2.pose.position.x = -5.0
        self.m2.pose.position.y = 0.0
        self.m2.pose.position.z = 0.5
        self.m2.pose.orientation.x = .707
        self.m2.pose.orientation.y = .707
        self.m2.pose.orientation.z = 0.0
        self.m2.pose.orientation.w = 0.0
        self.m2.color.r = 0.0
        self.m2.color.g = 0.0
        self.m2.color.b = 1.0
        self.m2.color.a = 1.0
        
        self.marker_array = MarkerArray()
        self.marker_array = [self.m1, self.m2]
        
        self.marker_array_pub(self.marker_array)

    
        
    
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
    