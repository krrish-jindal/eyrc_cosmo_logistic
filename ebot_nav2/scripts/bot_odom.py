#!/usr/bin/env python3

from geometry_msgs.msg import PoseStamped, Twist
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from ebot_docking.srv import DockSw
from linkattacher_msgs.srv import AttachLink, DetachLink
import yaml
import os
from arm_picky.srv import ArmNew 
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import Odometry
import time
import math
from geometry_msgs.msg import Quaternion
from tf_transformations import euler_from_quaternion
from tf_transformations import quaternion_from_euler
from geometry_msgs.msg import Polygon,Point32
from rclpy.parameter import Parameter


class OdomController(Node):

	def __init__(self):
		rclpy.init()  # Initialize rclpy here
		super().__init__('nav_dock',allow_undeclared_parameters=True,automatically_declare_parameters_from_overrides=True)
		

		self.vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
		self.vel_msg = Twist()
		self.odom_sub = self.create_subscription(Odometry, 'odom', self.odometry_callback, 10)



	def odometry_callback(self, msg):
	# Extract and update robot pose information from odometry message
		self.robot_pose[0] = msg.pose.pose.position.x
		self.robot_pose[1] = msg.pose.pose.position.y
		quaternion_array = msg.pose.pose.orientation
		orientation_list = [quaternion_array.x, quaternion_array.y, quaternion_array.z, quaternion_array.w]
		_, _, self.bot_yaw = euler_from_quaternion(orientation_list)

if __name__ == '__main__':
	
	s= OdomController()
