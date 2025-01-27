import unittest
import pytest
from launch import LaunchDescription
from launch_testing.actions import ReadyToTest
import rclpy
from launch_ros.actions import Node
import time
from geometry_msgs.msg import Twist
# Mark the launch description generation as a rostest
# Also, it's called generate_test_description() for a test
# But it still returns a LaunchDescription
@pytest.mark.rostest
def generate_test_description():
    # Create a node, as usual. But it's useful to keep it around
    node = Node(package='turtle_brick', executable='turtle_bot_node')
    # return a tuple (LaunchDescription, extra_dict)
    # extra_dict is a dictionary of parameter names -> values that get passed to
    # test cases.
    return (
        LaunchDescription([
            node, # here is where you put all launch actions you want to run during the test
                  # To test a particular launch file, include it
            ReadyToTest() # this is the last action. Can be used elsewhere somehow
            ]),
            {'myaction': node } # this is a way to pass the node action to the test case
            )
# The above returns the launch description. Now it's time for the test
# The goal is essentially to create a node that can then be used in all tests to
# call services and subscribe/publish messages
# unlike a regular node, it is often useful to not subclass node but rather
# just create it. It is also useful (and necessary) to spin_once() as needed
class TestMyTestCaseName(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        """Runs one time, when the testcase is loaded"""
        rclpy.init()
        

    @classmethod
    def tearDownClass(cls):
        """ Runs one time, when testcase is unloaded"""
        rclpy.shutdown()

    def setUp(self):
        """Runs before every test"""
        # so before every test, we create a new node
        self.node = rclpy.create_node('test_node')
        self.cmd_vel_subscriber = self.node.create_subscription(
            Twist, 
            '/cmd_vel', 
            self.cmd_vel_callback, 
            10
        )
        self.message_count = 0
        self.start_time = time.time()

    def tearDown(self):
        """Runs after every test"""
        # so after every test we destroy the node
        # Is a new node necessary for each test, or could we
        # create the nodes when we setupClass?
        self.node.destroy_node()
        
    def cmd_vel_callback(self, msg):
        self.message_count += 1
        

    def test_my_test1(self, launch_service, myaction, proc_output):
       """In UnitTest, any function starting with "test" is run as a test

          Args:
             launch_service - information about the launch
             myaction - this was passed in when we created the description
             proc_output - this object streams the output (if any) from the running process
       """
       # Here you can use self.node to publish and subscribe
       # launch_testing.WaitForTopics waits for something to be published on a topic
       # spin with rclpy.spin_once()
       # You can check and verify output with proc_output. launch/launch_testing has more information
       # You should specify a timeout for rclpy.spin_once() because if no ROS events happen it will
       # block forever (but if no events happen that is a bug and the test should fail
       expected_frequency = 100.0
       timeout = 2.0
       expected_message_count = expected_frequency * timeout
       
       end_time = time.time() + timeout
       
       while time.time() < end_time:
           rclpy.spin_once(self.node, timeout_sec=0.1)
           
        
       self.assertGreaterEqual(self.message_count, expected_message_count, 
            f"Expected at least {expected_message_count} messages but got {self.message_count}")
