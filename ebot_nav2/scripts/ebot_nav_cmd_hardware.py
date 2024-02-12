#!/usr/bin/env python3

from geometry_msgs.msg import PoseStamped, Twist
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from ebot_docking.srv import DockSw
from usb_relay.srv import RelaySw
from arm_picky.srv import ArmNew 
import yaml
import os
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import Odometry
import time
from tf_transformations import quaternion_from_euler
import math

class NavigationController(Node):

	def __init__(self):
		rclpy.init()  # Initialize rclpy here
		super().__init__('nav_dock')

		self.attach = self.create_client(srv_type=RelaySw, srv_name='/usb_relay_sw')
		self.arm_check = self.create_client(srv_type=ArmNew, srv_name='arm_control')
		self.client_docking = self.create_client(srv_type=DockSw, srv_name='dock_control')
		self.vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
		self.vel_msg = Twist()
		self.navigator = BasicNavigator()
		self.flag=False
		self.robot_pose = [0, 0]



	def send_request(self, orientation, rack_no):
		request_dock = DockSw.Request()
		request_dock.orientation_dock = True
		request_dock.orientation = orientation
		request_dock.rack_no = rack_no
		future = self.client_docking.call_async(request_dock)
		rclpy.spin_until_future_complete(self, future)
		return future.result()

	def arm_request(self, rack_no):
		request_arm = ArmNew.Request()
		request_arm.boom = True
		request_arm.rack_no = str(rack_no)
		gojo = self.arm_check.call_async(request_arm)
		rclpy.spin_until_future_complete(self, gojo)
		return gojo.result()

	def rack_attach(self, rack):
		req = RelaySw.Request()
		req.relaychannel = True
		req.relaystate = True

		atc = self.attach.call_async(req)
		rclpy.spin_until_future_complete(self, atc)
		self.vel_msg.linear.x = 0.2
		self.vel_pub.publish(self.vel_msg)
		time.sleep(2)
		self.vel_msg.linear.x = 0.0
		self.vel_pub.publish(self.vel_msg)
		return atc.result()

	def rack_detach(self, rack):
		req = RelaySw.Request()
		req.relaychannel = True
		req.relaystate = False

		atc = self.attach.call_async(req)
		rclpy.spin_until_future_complete(self, atc)
		self.vel_msg.linear.x = 0.2
		self.vel_pub.publish(self.vel_msg)
		time.sleep(2)
		self.vel_msg.linear.x = 0.0
		self.vel_pub.publish(self.vel_msg)
		return atc.result()

	def normalize_angle(self, angle):
		if angle < 0:
			angle = math.pi + (math.pi + angle)
		return angle
	
	def nav_coordinate(self,angle,x,y,poss):
		
		if poss=="final":
			d=0.65
		else:
			d=1.0
			
		self.a=x+(d*math.cos(angle))
		self.b=y+(d*math.sin(angle))
		return (self.a,self.b)
	
	def nav_theta(self,angle,obj_type):
		
		if obj_type =="rack":
			correct_angle=angle-1.57
		else:
			correct_angle=angle


		x,y,z,w=quaternion_from_euler(0,0,correct_angle)
		return (x,y,z,w)
	

	def nav_reach(self, goal):
		while not self.navigator.isTaskComplete():
			feedback = self.navigator.getFeedback()

			if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
				self.navigator.cancelTask()

		result = self.navigator.getResult()
		if result == TaskResult.SUCCEEDED:
			print(f'Goal {str(goal)} succeeded!')
		elif result == TaskResult.CANCELED:
			print('Goal was canceled!')
		elif result == TaskResult.FAILED:
			print('Goal failed!')
		else:
			print('Goal has an invalid return status!')



	def navigate_and_dock(self, goal_pick, goal_drop, goal_int, orientation_rack, rack,rack_no):
		self.rack_attach(rack)
		self.navigator.goToPose(goal_pick)
		self.nav_reach(goal_pick)

		self.send_request(orientation_rack, rack_no)
		#self.rack_attach(rack)

		self.navigator.goToPose(goal_int)
		self.nav_reach(goal_int)
		self.navigator.goToPose(goal_drop)
		self.nav_reach(goal_drop)
		self.rack_detach(rack)
		self.arm_request(rack_no = "3")


		
	# def navigate_and_dock(self, goal_pick, goal_drop, orientation_rack, rack,bot_coordinates):
	#     self.navigator.goToPose(goal_pick)
	#     self.nav_reach(goal_pick)
	#     # self.correct(bot_coordinates)
	#     print("POSE----",self.robot_pose)
	#     if self.flag == False:
	#         self.send_request(orientation_rack)
	#         self.rack_attach(rack)
	#         self.navigator.goToPose(goal_drop)
	#         self.nav_reach(goal_drop)
	#         self.rack_detach(rack)        


	def main(self):
		package_name = 'ebot_real_nav2'
		config = "config/config.yaml"

		ebot_nav2_dir = get_package_share_directory('ebot_real_nav2')

		pkg_share = FindPackageShare(package=package_name).find(package_name)
		config_path = os.path.join(pkg_share, config)
		with open(config_path, 'r') as infp:
			pos_rack = infp.read()

		data_dict = yaml.safe_load(pos_rack)

		positions = data_dict['position']
		rack1_coordinates = positions[0]['rack1']
		rack2_coordinates = positions[1]['rack2']
		rack3_coordinates = positions[2]['rack3']
		arm_coordinates=positions[3]['arm']
		package_id = data_dict['package_id'][0]

		orientation_rack_1 = rack1_coordinates[2]
		orientation_rack_2 = rack2_coordinates[2]
		orientation_rack_3 = rack3_coordinates[2]
		orientation_arm = arm_coordinates[2]
		rack_list = ["rack1", "rack2", "rack3"]
		
		theta_1=self.normalize_angle(orientation_rack_1)
		bot_pose_1=self.nav_coordinate(theta_1,rack1_coordinates[0],rack1_coordinates[1],"final")
		goal_theta_1= self.nav_theta(orientation_rack_1,"rack")

		theta_2=self.normalize_angle(orientation_rack_2)
		bot_pose_2=self.nav_coordinate(theta_2,rack2_coordinates[0],rack2_coordinates[1],"final")
		goal_theta_2= self.nav_theta(orientation_rack_2,"rack")

		theta_3=self.normalize_angle(orientation_rack_3)
		bot_pose_3=self.nav_coordinate(theta_3,rack3_coordinates[0],rack3_coordinates[1],"final")
		goal_theta_3= self.nav_theta(orientation_rack_3,"rack")


		theta_4=self.normalize_angle(orientation_arm)
		arm_pose_1=self.nav_coordinate(orientation_arm+90.0,arm_coordinates[0],arm_coordinates[1],"final")
		goal_theta_4= self.nav_theta(orientation_arm+90.0,"arm")

		theta_5=self.normalize_angle(orientation_arm)
		arm_pose_2=self.nav_coordinate(orientation_arm,arm_coordinates[0],arm_coordinates[1],"final")
		goal_theta_5= self.nav_theta(orientation_arm,"arm")

		theta_6=self.normalize_angle(orientation_arm)
		arm_pose_3=self.nav_coordinate(orientation_arm-90.0,arm_coordinates[0],arm_coordinates[1],"final")
		goal_theta_6= self.nav_theta(orientation_arm-90.0,"arm")

		init_arm_pose_1=self.nav_coordinate(orientation_arm+90.0,arm_coordinates[0],arm_coordinates[1],"initial")
		init_goal_theta_4= self.nav_theta(orientation_arm+90.0,"arm")

		init_arm_pose_2=self.nav_coordinate(orientation_arm,arm_coordinates[0],arm_coordinates[1],"initial")
		init_goal_theta_5= self.nav_theta(orientation_arm,"arm")

		init_arm_pose_3=self.nav_coordinate(orientation_arm-90.0,arm_coordinates[0],arm_coordinates[1],"initial")
		init_goal_theta_6= self.nav_theta(orientation_arm-90.0,"arm")

		goal_pick_1 = PoseStamped()
		goal_pick_1.header.frame_id = 'map'
		goal_pick_1.header.stamp = self.navigator.get_clock().now().to_msg()
		goal_pick_1.pose.position.x = bot_pose_1[0]
		goal_pick_1.pose.position.y = bot_pose_1[1]
		goal_pick_1.pose.orientation.x = goal_theta_1[0]
		goal_pick_1.pose.orientation.y = goal_theta_1[1]
		goal_pick_1.pose.orientation.z = goal_theta_1[2]
		goal_pick_1.pose.orientation.w = goal_theta_1[3]
	
		goal_pick_2 = PoseStamped()
		goal_pick_2.header.frame_id = 'map'
		goal_pick_2.pose.position.x = bot_pose_2[0]
		goal_pick_2.pose.position.y = bot_pose_2[1]
		goal_pick_2.pose.orientation.x = goal_theta_2[0]
		goal_pick_2.pose.orientation.y = goal_theta_2[1]
		goal_pick_2.pose.orientation.z = goal_theta_2[2]
		goal_pick_2.pose.orientation.w = goal_theta_2[3]

		
		goal_pick_3 = PoseStamped()
		goal_pick_3.header.frame_id = 'map'
		goal_pick_3.header.stamp = self.navigator.get_clock().now().to_msg()
		goal_pick_3.pose.position.x = bot_pose_3[0]		
		goal_pick_3.pose.position.y = bot_pose_3[1]
		goal_pick_3.pose.orientation.x = goal_theta_3[0]
		goal_pick_3.pose.orientation.y = goal_theta_3[1]
		goal_pick_3.pose.orientation.z = goal_theta_3[2]
		goal_pick_3.pose.orientation.w = goal_theta_3[3]


		goal_drop_init_1 = PoseStamped()
		goal_drop_init_1.header.frame_id = 'map'
		goal_drop_init_1.header.stamp = self.navigator.get_clock().now().to_msg()
		goal_drop_init_1.pose.position.x = init_arm_pose_1[0]
		goal_drop_init_1.pose.position.y = init_arm_pose_1[1]
		goal_drop_init_1.pose.orientation.x = init_goal_theta_4[0]
		goal_drop_init_1.pose.orientation.y = init_goal_theta_4[1]
		goal_drop_init_1.pose.orientation.z = init_goal_theta_4[2]
		goal_drop_init_1.pose.orientation.w = init_goal_theta_4[3]
	
		goal_drop_init_2 = PoseStamped()
		goal_drop_init_2.header.frame_id = 'map'
		goal_drop_init_2.pose.position.x = init_arm_pose_2[0]
		goal_drop_init_2.pose.position.y = init_arm_pose_2[1]
		goal_drop_init_2.pose.orientation.x = init_goal_theta_5[0]
		goal_drop_init_2.pose.orientation.y = init_goal_theta_5[1]
		goal_drop_init_2.pose.orientation.z = init_goal_theta_5[2]
		goal_drop_init_2.pose.orientation.w = init_goal_theta_5[3]

		goal_drop_init_3 = PoseStamped()
		goal_drop_init_3.header.frame_id = 'map'
		goal_drop_init_3.pose.position.x = init_arm_pose_3[0]
		goal_drop_init_3.pose.position.y = init_arm_pose_3[1]
		goal_drop_init_3.pose.orientation.x = init_goal_theta_6[0]
		goal_drop_init_3.pose.orientation.y = init_goal_theta_6[1]
		goal_drop_init_3.pose.orientation.z = init_goal_theta_6[2]
		goal_drop_init_3.pose.orientation.w = init_goal_theta_6[3]

		goal_drop_1 = PoseStamped()
		goal_drop_1.header.frame_id = 'map'
		goal_drop_1.header.stamp = self.navigator.get_clock().now().to_msg()
		goal_drop_1.pose.position.x = arm_pose_1[0]
		goal_drop_1.pose.position.y = arm_pose_1[1]
		goal_drop_1.pose.orientation.x = goal_theta_4[0]
		goal_drop_1.pose.orientation.y = goal_theta_4[1]
		goal_drop_1.pose.orientation.z = goal_theta_4[2]
		goal_drop_1.pose.orientation.w = goal_theta_4[3]
	
		goal_drop_2 = PoseStamped()
		goal_drop_2.header.frame_id = 'map'
		goal_drop_2.pose.position.x = arm_pose_2[0]
		goal_drop_2.pose.position.y = arm_pose_2[1]
		goal_drop_2.pose.orientation.x = goal_theta_5[0]
		goal_drop_2.pose.orientation.y = goal_theta_5[1]
		goal_drop_2.pose.orientation.z = goal_theta_5[2]
		goal_drop_2.pose.orientation.w = goal_theta_5[3]

		goal_drop_3 = PoseStamped()
		goal_drop_3.header.frame_id = 'map'
		goal_drop_3.pose.position.x = arm_pose_3[0]
		goal_drop_3.pose.position.y = arm_pose_3[1]
		goal_drop_3.pose.orientation.x = goal_theta_6[0]
		goal_drop_3.pose.orientation.y = goal_theta_6[1]
		goal_drop_3.pose.orientation.z = goal_theta_6[2]
		goal_drop_3.pose.orientation.w = goal_theta_6[3]

		self.navigator.waitUntilNav2Active()

		self.arm_request(rack_no = "0")
		self.navigate_and_dock(goal_pick_3, goal_drop_2, goal_drop_init_2, orientation_rack_3, rack_list[2], "3")
		self.arm_request(rack_no = "1")


		exit(0)

if __name__ == '__main__':
	nav_controller = NavigationController()
	nav_controller.main()