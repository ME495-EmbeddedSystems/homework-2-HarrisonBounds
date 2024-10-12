import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
import rclpy.parameter
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped, Twist, PoseStamped
from nav_msgs.msg import Odometry
from turtlesim.msg import Pose


class Turtle_Robot_Node(Node):
    def __init__(self):
        super().init('turtle_bot_node')
        cb_group = MutuallyExclusiveCallbackGroup()
        
        self.frequency = 100
        self.interval = 1 / self.frequency
        
        self.static_broadcaster = StaticTransformBroadcaster(self)
        
        world_base_tf = TransformStamped()
        world_base_tf.header.stamp = self.get_clock().now().to_msg()
        world_base_tf.header.frame_id = 'world'
        world_base_tf.child_frame_id = 'base'
        
        self.static_broadcaster.sendTransform(world_base_tf)
        
        self.broadcaster = TransformBroadcaster(self)
        
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10, callback_group=cb_group)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10, callback_group=cb_group)
        self.cmd_vel_pub = self.create_publisher(Twist, '/turtlesim1/turtle1/cmd_vel', 10, callback_group=cb_group)
        
        self.goal_sub = self.create_subscription(PoseStamped, 'goal_pose', self.goal_callback, 10, callback_group=cb_group)
        #Create tilt subscriber??
        self.pose_subscription = self.create_subscription(Pose, '/turtlesim1/turtle1/pose', self.sub_callback, 10, callback_group=cb_group)
        
        self._tmr = self.create_timer(self.interval, self.timer_callback)
        
        
    def timer_callback(self):
        #Calculate and publish the joint states
        
        #Broadcast base_link frame relative to odom frame
        
        #Publish Odometry message
        
        #Publish cmd_vel
        
        
        pass
    
    def goal_callback(self):
        pass
    
def main(args=None):
    """Entrypoint for the mynode ROS node."""
    rclpy.init(args=args)
    node = Turtle_Robot_Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    import sys
    main(sys.argv)
    
    