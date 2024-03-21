#!/usr/bin/env python3


import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from tf_transformations import euler_from_quaternion
from ebot_docking.srv import DockSw  # Import custom service message
import math, statistics
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os
import yaml
import time
# Define a class for your ROS2 node
class MyRobotDockingController(Node):

	def __init__(self):
		# Initialize the ROS2 node with a unique name
		super().__init__('my_robot_docking_controller')

		# Create a callback group for managing callbacks
		self.callback_group = ReentrantCallbackGroup()

		self.get_logger().info("Server Started")


		# Subscribe to odometry data for robot pose information
		self.odom_sub = self.create_subscription(Odometry, 'odom', self.odometry_callback, 10)

		# Subscribe to ultrasonic sensor data for distance measurements
		self.ultrasonic_rl_sub = self.create_subscription(Range, '/ultrasonic_rl/scan', self.ultrasonic_rl_callback, 10)
		# Add another one here
		self.ultrasonic_rr_sub = self.create_subscription(Range, '/ultrasonic_rr/scan', self.ultrasonic_rr_callback, 10)


		# Create a ROS2 service for controlling docking behavior, can add another custom service message
		self.dock_control_srv = self.create_service(DockSw, 'dock_control', self.dock_control_callback, callback_group=self.callback_group)

		# Create a publisher for sending velocity commands to the robot
		self.vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)

		# Initialize all  flags and parameters here
		self.is_docking = False
		self.robot_pose = [0,0,0] 
		self.dock_aligned = False
		self.kp = 1.0
		self.normalize_yaw_bot = 0
		self.normalize_yaw_rack = 0
		self.difference = 0
		package_name = 'ebot_nav2'
		config = "config/config_sim.yaml"
		self.flag =0

		pkg_share = FindPackageShare(package=package_name).find(package_name)
		config_path = os.path.join(pkg_share, config)
		with open(config_path, 'r') as infp:
			pos_rack = infp.read()

		data_dict = yaml.safe_load(pos_rack)
		self.x_pose = []
		positions = data_dict['position']
		self.rack1_coordinates = positions[0]['rack1']
		self.x_pose.append(self.rack1_coordinates)
		self.rack2_coordinates = positions[1]['rack2']
		self.x_pose.append(self.rack2_coordinates)
		self.rack3_coordinates = positions[2]['rack3']
		self.x_pose.append(self.rack3_coordinates)

		self.rack_yaw=[]

		orientation_rack_1 = self.rack1_coordinates[2]
		self.rack_yaw.append(orientation_rack_1)

		orientation_rack_2 = self.rack2_coordinates[2]
		self.rack_yaw.append(orientation_rack_2)

		orientation_rack_3 = self.rack3_coordinates[2]
		self.rack_yaw.append(orientation_rack_3)


		self.controller_timer = self.create_timer(0.1, self.controller_loop)

	# Callback function for odometry data
	def odometry_callback(self, msg):
		# Extract and update robot pose information from odometry message
		self.robot_pose[0] = msg.pose.pose.position.x
		self.robot_pose[1] = msg.pose.pose.position.y
		quaternion_array = msg.pose.pose.orientation
		orientation_list = [quaternion_array.x, quaternion_array.y, quaternion_array.z, quaternion_array.w]
		_, _, yaw = euler_from_quaternion(orientation_list)
		
		self.robot_pose[2] = yaw

	# Callback function for the left ultrasonic sensor
	def ultrasonic_rl_callback(self, msg):
		self.usrleft_value = msg.range

	# Callback function for the right ultrasonic sensor
	def ultrasonic_rr_callback(self, msg):
		self.usrright_value = msg.range

	# Utility function to normalize angles within the range of 0 to 2π 
	def normalize_angle(self, angle):
		if angle < 0:
			angle = math.pi + (math.pi + angle)
		return angle


	# Main control loop for managing docking behavior

	def controller_loop(self):
		vel = Twist()

		if self.is_docking:
  
			print("After")
			self.difference = self.normalize_yaw_rack - self.normalize_yaw_bot

			if self.orientation_dock ==True:

				print("YAW------------YAW",(self.rack_yaw[int(self.rack_no)-1]))


				error = self.x_pose[int(self.rack_no) - 1][0] - self.robot_pose[0]
				error2 = self.x_pose[int(self.rack_no) - 1][1] - self.robot_pose[1]
				robot_head=str(self.robot_pose[2]/abs(self.robot_pose[2]))
				print("ERRORS------",error,error2)




#   X DIRECTION POSS CORRECTION
				
				if 0.3 > abs(self.x_pose[int(self.rack_no) - 1][0] - self.robot_pose[0]) > 0.025  and robot_head== "1.0":

					print(self.robot_pose[0],self.robot_pose[1],"---------44444444",abs(self.x_pose[int(self.rack_no) - 1][0] - self.robot_pose[0]))
					print("=============================================")

					vel.linear.x = -error *0.4
					self.vel_pub.publish(vel)

				elif 0.3 > abs(self.x_pose[int(self.rack_no) - 1][0] - self.robot_pose[0]) > 0.025 and robot_head== "-1.0":
					print(self.robot_pose[0],self.robot_pose[1],"---------44444444",abs(self.x_pose[int(self.rack_no) - 1][0] - self.robot_pose[0]))
					vel.linear.x = error *0.4
					self.vel_pub.publish(vel)



#   Y DIRECTION POSS CORRECTION

				elif 0.3 > abs(self.x_pose[int(self.rack_no) - 1][1] - self.robot_pose[1]) > 0.025  and robot_head== "1.0":
					print(self.robot_pose[0],self.robot_pose[1],"---------33333333333",abs(self.x_pose[int(self.rack_no) - 1][1] - self.robot_pose[1]))
					vel.linear.x = error2 *0.4
					self.vel_pub.publish(vel)
		
				
				elif 0.3 > abs(self.x_pose[int(self.rack_no) - 1][1] - self.robot_pose[1]) > 0.025  and robot_head== "-1.0":
					print(self.robot_pose[0],self.robot_pose[1],"---------33333333333",abs(self.x_pose[int(self.rack_no) - 1][1] - self.robot_pose[1]))
					vel.linear.x = -error2 *0.4
					self.vel_pub.publish(vel)


#   DESIRE POSS REACH 
					
				else:

					print("Before")
					vel.linear.x = 0.0
					self.vel_pub.publish(vel)
					self.flag = 1
					self.orientation_dock = False



# START ORIENTATION CORRECTION
					
			elif self.flag == 1:
				if abs(self.difference) > 0.1:
					if self.difference > 0:
						vel.angular.z = self.difference *0.6 
					else:
						vel.angular.z = self.difference *0.6 
					self.vel_pub.publish(vel)



#  ORIENTATION CORRECTED
					
				else:
					vel.angular.z = 0.0
					self.vel_pub.publish(vel)
					self.orientation_dock = False
					self.linear_dock = False
					print("successfully oriented",self.linear_dock)
					self.flag = 0

			


#  LINEAR DOCKING STARTS
					
			elif self.linear_dock == False:
				print("FLAG---",self.flag)
				print("++++++++++++++++++++++++++++++++++++")
				self.usrdiff=self.usrleft_value-self.usrright_value
				# print("L-",self.usrleft_value,"R-",self.usrright_value)
				# print("Diff---",self.difference)
				self.diff=self.usrleft_value-self.usrright_value




#   CORRECTING ULTRA SONIC DISTANCE ERROR
				
				if self.usrleft_value > 0.15 and round(self.usrright_value,1) != round(self.usrleft_value,1):



	#  CORRECTING ULRA SONIC ERROR 
					
					if abs(self.difference)<=0.02:
						print(">>>>>=====>>>>>>")
						vel.angular.z = self.usrdiff*0.2 

						self.vel_pub.publish(vel)
				
	# CORRECTING RACK & BOT YAW ERROR

					else:
						print(">>>>>>>>>>>>>>>")
						vel.angular.z = self.difference*0.2
						vel.linear.x = -0.2

						self.vel_pub.publish(vel)

 
					# vel.linear.x = -self.usrleft_value * 0.4
					self.orientation_dock = False
					self.linear_dock = False



#   NO ULTRA SONIC DISTANCE ERROR DIRECT DOCKING
					
				elif self.usrleft_value > 0.15 and round(self.usrright_value,1) == round(self.usrleft_value,1):
					print("===============")
					self.orientation_dock = False
					vel.linear.x = -self.usrleft_value * 0.4
					vel.angular.z = 0.0
					self.vel_pub.publish(vel)
					self.linear_dock = False


#  DOCKING COMPLEATED
					
				else:
					vel.linear.x = 0.0
					self.vel_pub.publish(vel)
					print("docking done")
					self.linear_dock = True
					self.is_docking = False
					self.dock_aligned = True

				

	# Callback function for the DockControl service
	def dock_control_callback(self, request, response):
		# Extract desired docking parameters from the service request
		self.linear_dock = request.linear_dock
		self.orientation_dock = request.orientation_dock
		self.distance = request.distance
		self.orientation = request.orientation
		self.rack_no = request.rack_no
		# print(self.orientation)

		self.normalize_yaw_rack = self.normalize_angle(self.orientation)
		# print(self.normalize_yaw_rack)
		self.normalize_yaw_bot = self.normalize_angle(self.robot_pose[2])
		# print(self.normalize_yaw_bot)


		# Reset flags and start the docking process
		self.dock_aligned = False
		print(self.dock_aligned)

		# Log a message indicating that docking has started
		self.get_logger().info("Docking started!")
		# Create a rate object to control the loop frequency
		self.rate = self.create_rate(2, self.get_clock())

		# Wait until the robot is aligned for docking
		while not self.dock_aligned:
			self.normalize_yaw_rack = self.normalize_angle(self.orientation)
			self.normalize_yaw_bot = self.normalize_angle(self.robot_pose[2])
			self.is_docking = True
			self.get_logger().info("Waiting for alignment...")
			self.controller_loop()
			# self.rate.sleep()

		# Set the service response indicating success
		response.success = True
		response.message = "Docking control initiated"
		return response

# Main function to initialize the ROS2 node and spin the executor
def main(args=None):
	rclpy.init(args=args)

	my_robot_docking_controller = MyRobotDockingController()

	executor = MultiThreadedExecutor()
	executor.add_node(my_robot_docking_controller)

	executor.spin()

	my_robot_docking_controller.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()