import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
import rclpy.parameter
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped, Twist, PoseStamped, Vector3
from nav_msgs.msg import Odometry
from turtlesim.msg import Pose
import math
import numpy as np



class Turtle_Robot_Node(Node):
    def __init__(self):
        super().__init__('turtle_bot_node')
        
        self.frequency = 100
        self.interval = 1 / self.frequency
        
        qos = QoSProfile(depth=10, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.wheel_radius = 0.04 #Change so that this is dynamic!
        self.forward_velocity = 1.0
        self.max_velocity = 5.0
        self.min_velocity = 0.5
        self.angle = 0.0
        self.old_time = self.get_clock().now().nanoseconds
        self.old_time = self.old_time / 1e9 #Transform nanoseconds to seconds
        
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/turtlesim1/turtle1/cmd_vel', 10)
        
        self.goal_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        #Create tilt subscriber??
        self.pose_subscription = self.create_subscription(Pose, '/turtlesim1/turtle1/pose', self.pose_callback, 10)
        self.delta_x = 0.0
        self.delta_y = 0.0
        self.delta_theta = 0.0
        self.pose = 0.0
        
        self._tmr = self.create_timer(self.interval, self.timer_callback)
        
        #Create the odometry static frame
        self.odomX = 5.54
        self.odomY = 5.54
        self.static_broadcaster = StaticTransformBroadcaster(self, qos)
        
        self.world_odom_tf = TransformStamped()
        self.world_odom_tf.header.stamp = self.get_clock().now().to_msg()
        self.world_odom_tf.header.frame_id = "world"
        self.world_odom_tf.child_frame_id = "odom"
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
        

        self.static_broadcaster.sendTransform(self.world_odom_tf)
        
        self.broadcaster = TransformBroadcaster(self, qos)
        
        
    def timer_callback(self):
        #Publish joint states
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.header.frame_id = "odom"
        joint_state_msg.name = ["j1", "j2", "j3"]
        joint_state_msg.position = [0.0, 0.0, 0.0]
        joint_state_msg.velocity = [0.0, 0.0, 0.0]

        # self.joint_state_pub.publish(joint_state_msg)
       
        #Publish odom to base_link transform
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
        
        
        #Publish cmd_vel msg if the robot hasnt reached the goal
        if self.goal_x != None and self.goal_y != None and self.goal_x != self.delta_x and self.goal_y != self.delta_y:
            self.angle = math.atan2(self.goal_y - self.delta_y, self.goal_x - self.delta_x)
            self.distance_to_goal = math.sqrt((self.goal_y - self.delta_y)**2 + (self.goal_x - self.delta_x)**2)
            move = self.forward_velocity * self.distance_to_goal
            
            if self.angle > np.pi:
                self.angle = self.angle - 2*np.pi
            elif self.angle < -np.pi:
                self.angle = self.angle + 2*np.pi
        
            if move > self.max_velocity:
                move = self.max_velocity
            elif move < self.min_velocity:
                move = self.min_velocity
        
            cmd_vel_msg = self.move_turtle(move * math.cos(self.angle), move * math.sin(self.angle))
            self.cmd_vel_pub.publish(cmd_vel_msg)
            
        else:
            cmd_vel_msg = self.move_turtle(0, 0)
            self.cmd_vel_pub.publish(cmd_vel_msg)
            
        
        # Publish Odometry message
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

    
    def move_turtle(self, xdot, ydot):
        """ Create a twist suitable for a turtle

            Args:
            xdot (float) : the forward velocity
            omega (float) : the angular velocity

            Returns:
            Twist - a 2D twist object corresponding to linear/angular velocity
        """
        return Twist(linear = Vector3(x = float(xdot), y = float(ydot), z = 0.0),
                    angular = Vector3(x = 0.0, y = 0.0, z = 0.0))
        
    def pose_callback(self, msg):
        self.delta_x = msg.x
        self.delta_y = msg.y
        self.delta_theta = msg.theta
        
    def goal_callback(self, msg):
        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y
    
        
def quaternion_from_euler(ai, aj, ak):
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
    """Entrypoint for the mynode ROS node."""
    rclpy.init(args=args)
    node = Turtle_Robot_Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    import sys
    main(sys.argv)
    
    