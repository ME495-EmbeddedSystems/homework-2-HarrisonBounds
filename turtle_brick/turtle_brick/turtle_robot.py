import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
import rclpy.parameter
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped, Twist, PoseStamped, Vector3
from nav_msgs.msg import Odometry
from turtlesim.msg import Pose
import math



class Turtle_Robot_Node(Node):
    def __init__(self):
        super().__init__('turtle_bot_node')
        cb_group = MutuallyExclusiveCallbackGroup()
        
        self.frequency = 100
        self.interval = 1 / self.frequency
        
        #Create the odometry static frame
        self.static_broadcaster = StaticTransformBroadcaster(self)
        
        world_odom_tf = TransformStamped()
        world_odom_tf.header.stamp = self.get_clock().now().to_msg()
        world_odom_tf.header.frame_id = 'world'
        world_odom_tf.child_frame_id = 'odom'
        self.static_broadcaster.sendTransform(world_odom_tf)
        
        self.broadcaster = TransformBroadcaster(self)
        
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10, callback_group=cb_group)
        self.joint_names = ['wheel_to_base', 'base_to_platform', 'base_to_stem', 'stem_to_wheel']
        self.joint_positions = [0.0, 0.0, 0.0, 0.0]
        self.joint_velocities = [0.0, 0.0, 0.0, 0.0]
        self.wheel_radius = 0.04 #Change so that this is dynamic!
        self.forward_velocity = 1.0
        self.caster_wheel_angle = 0.0
        self.old_time = self.get_clock().now().nanoseconds
        self.old_time = self.old_time / 1e9 #Transform nanoseconds to seconds
        
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10, callback_group=cb_group)
        self.cmd_vel_pub = self.create_publisher(Twist, '/turtlesim1/turtle1/cmd_vel', 10, callback_group=cb_group)
        
        self.goal_sub = self.create_subscription(PoseStamped, 'goal_pose', self.goal_callback, 10, callback_group=cb_group)
        #Create tilt subscriber??
        self.pose_subscription = self.create_subscription(Pose, '/turtlesim1/turtle1/pose', self.pose_callback, 10, callback_group=cb_group)
        self.delta_x = 0.0
        self.delta_y = 0.0
        self.delta_theta = 0.0
        
        self._tmr = self.create_timer(self.interval, self.timer_callback)
        
        
    def timer_callback(self):
        #Calculate and publish the joint states
        current_time = self.get_clock().now().nanoseconds 
        current_time = current_time / 1e9 
        dt = current_time - self.old_time
        self.old_time = current_time
        
        angular_velocity = self.forward_velocity / self.wheel_radius
        
        self.joint_positions[0] = 0.0
        self.joint_positions[1] = 0.0
        self.joint_positions[2] = 0.0
        self.joint_positions[3] = angular_velocity * dt
     
        self.joint_velocities[0] = 0.0
        self.joint_velocities[1] = 0.0
        self.joint_velocities[2] = 0.0
        self.joint_velocities[3] = angular_velocity
        

        #Publish wheel joint state
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = self.joint_names
        joint_state_msg.position = self.joint_positions
        joint_state_msg.velocity = self.joint_velocities
        self.joint_state_pub.publish(joint_state_msg)
        
        #Broadcast base_link frame relative to odom frame
        odom_base_link = TransformStamped()
        odom_base_link.header.frame_id = 'odom'
        odom_base_link.child_frame_id = 'base_link'
        odom_base_link.transform.translation.x = self.delta_x
        odom_base_link.transform.translation.y = self.delta_y
        odom_base_link.transform.translation.z = 0.0
        
        odom_base_link.transform.rotation.x = 0.0
        odom_base_link.transform.rotation.y = 0.0
        odom_base_link.transform.rotation.z = self.delta_theta
        odom_base_link.transform.rotation.w = math.cos(self.delta_theta / 2.0)

        self.broadcaster.sendTransform(odom_base_link)
        
        #Use Quaternion
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.delta_x
        odom.pose.pose.position.y = self.delta_y
        odom.pose.pose.position.z = self.delta_z
        odom.pose.pose.orientation = odom_base_link.transform.rotation
        self.odom_pub.publish(odom)
        
        #Publish Twist
        twist = self.move_turtle(self.forward_velocity, 0.5)
        self.cmd_vel_pub.publish(twist)
        

        #Publish Odometry message
        
        #Publish cmd_vel
        
        
    def move_turtle(self, xdot, omega):
        """ Create a twist suitable for a turtle

            Args:
            xdot (float) : the forward velocity
            omega (float) : the angular velocity

            Returns:
            Twist - a 2D twist object corresponding to linear/angular velocity
        """
        return Twist(linear = Vector3(x = float(xdot), y = 0.0, z = 0.0),
                    angular = Vector3(x = 0.0, y = 0.0, z = float(omega)))
        
    
    def goal_callback(self, msg):
        #Drive towards goal
        pass
    
    def pose_callback(self, msg):
        self.delta_x = msg.x
        self.delta_y = msg.y
        self.delta_theta = msg.theta
        
    
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
    
    