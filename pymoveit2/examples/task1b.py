#!/usr/bin/env python3

from os import path
from threading import Thread

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
import time
from pymoveit2 import MoveIt2
from pymoveit2.robots import ur5
import math
import sys
import tf2_ros
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from scipy.spatial.transform import Rotation as R
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Quaternion
from tf_transformations import quaternion_from_euler
import tf_transformations
from geometry_msgs.msg import TwistStamped
from linkattacher_msgs.srv import AttachLink
from linkattacher_msgs.srv import DetachLink
from math import cos, sin
import math, time
from copy import deepcopy
import rclpy
import tf2_ros
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from pymoveit2.robots import ur5
from rclpy.qos import (
	QoSDurabilityPolicy,
	QoSHistoryPolicy,
	QoSProfile,
	QoSReliabilityPolicy,
)

RACK_MESH = path.join(
	path.dirname(path.realpath(__file__)), "assets", "rack1.stl"
)
BASE_MESH = path.join(
	path.dirname(path.realpath(__file__)), "assets", "arm_base_40.stl"
)

endf_x = 0
endf_y = 0
endf_z = 0


pos1 = [0.35, 0.1, 0.68]
pos2 = [0.35, 0.1, 0.68]
pos3 = [0.35, 0.1, 0.68]

def truncate(f, n):
	return math.floor(f * 10 ** n) / 10 ** n
class endf(Node):
	def __init__(self):
		super().__init__('move_ur5') 
		self.tf_buffer = tf2_ros.buffer.Buffer()                                        # buffer time used for listening transforms
		self.listener = tf2_ros.TransformListener(self.tf_buffer, self)
		self.br = tf2_ros.TransformBroadcaster(self)
		self.switch = False
		self.twist_pub = self.create_publisher(TwistStamped, "/servo_node/delta_twist_cmds", 10)
		self.gripper_control = self.create_client(AttachLink, '/GripperMagnetON')
		self.gripper_control_off = self.create_client(DetachLink, '/GripperMagnetOFF')
		self.callback_group = ReentrantCallbackGroup()
		self.pz = 0

		self.moveit2 = MoveIt2(
		node=self,
		joint_names=ur5.joint_names(),
		base_link_name=ur5.base_link_name(),
		end_effector_name=ur5.end_effector_name(),
		group_name=ur5.MOVE_GROUP_ARM,
		callback_group=self.callback_group,)
		self.declare_parameter(
        "joint_positions_initial",
        [
            0.0,
            -2.39,
            2.4,
            -3.15,
            -1.58,
            3.15
        ],
    )
		self.declare_parameter(
        "joint_positions_positive",
        [
            1.57,
            -2.39,
            2.4,
            -3.15,
            -1.58,
            3.15
        ],
    )
		self.declare_parameter(
        "joint_positions_negative",
        [
            -1.57,
            -2.39,
            2.4,
            -3.15,
            -1.58,
            3.15
        ],
    )
		self.declare_parameter(
        "joint_positions_final_1",
        [
            0.0,
            -2.07694,
            -0.418879,
            -3.26377,
            -1.41372,
            3.05432
        ],
    )
		self.declare_parameter(
        "joint_positions_final_2",
        [
            0.541052,
            -2.35619,
            -0.698132,
            -3.14159,
            -1.58825,
            3.14159
        ],
    )
		self.declare_parameter(
        "joint_positions_final_3",
        [
            0.244346,
            -2.16921,
            -0.820305,
            -3.14159,
            -1.58825,
            3.14159
        ],
    )

	def servo(self, box_no):
		while rclpy.ok():
			try:
				tool0 = self.tf_buffer.lookup_transform('base_link', "tool0", rclpy.time.Time())
				box49 = self.tf_buffer.lookup_transform('base_link', f"obj_{box_no}", rclpy.time.Time())
				


				# while rclpy.ok():
				# 	try:
				# 		tool0 = enftf.tf_buffer.lookup_transform('base_link', "tool0", rclpy.time.Time())
				# 		box49 = enftf.tf_buffer.lookup_transform('base_link', "obj_49", rclpy.time.Time())
				# 	except:
				# 		pass


				# 	roll_box , pitch_box, yaw_box = tf_transformations.euler_from_quaternion([box49.transform.rotation.x, box49.transform.rotation.y, box49.transform.rotation.z, box49.transform.rotation.w])
				# 	print(yaw_box)
				# 	roll_tool , pitch_tool, yaw_tool = tf_transformations.euler_from_quaternion([tool0.transform.rotation.x, tool0.transform.rotation.y, tool0.transform.rotation.z, tool0.transform.rotation.w])

				# 	print(f"y ={round((yaw_tool) - (yaw_box),4)}")

				# 	if round((yaw_tool) - (yaw_box),4) < 0.003:
				# 		__twist_msg = TwistStamped()
				# 		__twist_msg.header.stamp = enftf.get_clock().now().to_msg()
				# 		__twist_msg.header.frame_id = ur5.base_link_name()
				# 		__twist_msg.twist.angular.z = round((yaw_tool) - (yaw_box),4) *2
				# 		__twist_pub.publish(__twist_msg)
				# 	else:
				# 		break




				while rclpy.ok():
					try:
						box49 = self.tf_buffer.lookup_transform('base_link', f"obj_{box_no}", rclpy.time.Time())
						tool0 = self.tf_buffer.lookup_transform('base_link', "tool0", rclpy.time.Time())
					except:
						pass
					print(f"y ={round((box49.transform.translation.y) - (tool0.transform.translation.y),4)}")
					print(f"x ={round((box49.transform.translation.x) - (tool0.transform.translation.x),4)}")
					print(f"z ={round((box49.transform.translation.z) - (tool0.transform.translation.z),4)}")
					joint_positions_initial = (self.get_parameter("joint_positions_initial").get_parameter_value().double_array_value)
					joint_positions_final_1 = (self.get_parameter("joint_positions_final_1").get_parameter_value().double_array_value)
					joint_positions_final_2 = (self.get_parameter("joint_positions_final_2").get_parameter_value().double_array_value)
					joint_positions_final_3 = (self.get_parameter("joint_positions_final_3").get_parameter_value().double_array_value)

					if round((box49.transform.translation.y) - (tool0.transform.translation.y),4) > 0.003 or round((box49.transform.translation.x) - (tool0.transform.translation.x),4) > 0.003 or round((box49.transform.translation.z) - (tool0.transform.translation.z),4) > 0.003:
						__twist_msg = TwistStamped()
						__twist_msg.header.stamp = self.get_clock().now().to_msg()
						__twist_msg.header.frame_id = ur5.base_link_name()
						__twist_msg.twist.linear.y = round((box49.transform.translation.y) - (tool0.transform.translation.y),4) *2
						__twist_msg.twist.linear.x = round((box49.transform.translation.x) - (tool0.transform.translation.x),4) *2
						__twist_msg.twist.linear.z = round((box49.transform.translation.z) - (tool0.transform.translation.z),4) *2
						self.twist_pub.publish(__twist_msg)
					else :
						while not self.gripper_control.wait_for_service(timeout_sec=1.0):
							self.get_logger().info('EEF service not available, waiting again...')
						print("tryiing to attach")
						req = AttachLink.Request()
						req.model1_name =  f'box{box_no}'      
						req.link1_name  = 'link'       
						req.model2_name =  'ur5'       
						req.link2_name  = 'wrist_3_link'
						self.gripper_control.call_async(req)
						break
				while rclpy.ok():
					try:
						box49 = self.tf_buffer.lookup_transform('base_link', f"obj_{box_no}", rclpy.time.Time())
						tool0 = self.tf_buffer.lookup_transform('base_link', "tool0", rclpy.time.Time())
						print(f"x = {round(tool0.transform.translation.x,2)}")
						print(f"y = {round(tool0.transform.translation.x,2)}")

					except:
						pass
					if box_no == str(1) and (round(tool0.transform.translation.x,2) > 0.21):
						__twist_msg = TwistStamped()
						__twist_msg.header.stamp = self.get_clock().now().to_msg()
						__twist_msg.header.frame_id = ur5.base_link_name()
						__twist_msg.twist.linear.z = -0.04
						__twist_msg.twist.linear.x = -0.4
						self.twist_pub.publish(__twist_msg)
					elif box_no == str(49) and (round(tool0.transform.translation.y,2) > 0.21):
						__twist_msg = TwistStamped()
						__twist_msg.header.stamp = self.get_clock().now().to_msg()
						__twist_msg.header.frame_id = ur5.base_link_name()
						__twist_msg.twist.linear.z = 0.04
						__twist_msg.twist.linear.y = -0.2
						self.twist_pub.publish(__twist_msg)
					elif box_no == str(3) and (round(tool0.transform.translation.y,2) < -0.22):
						__twist_msg = TwistStamped()
						__twist_msg.header.stamp = self.get_clock().now().to_msg()
						__twist_msg.header.frame_id = ur5.base_link_name()
						__twist_msg.twist.linear.z = 0.04
						__twist_msg.twist.linear.y = 0.2
						__twist_msg.twist.linear.x = -0.2
						self.twist_pub.publish(__twist_msg)
					else:
						break
				if self.pz == 0:
					self.moveit2.move_to_configuration(joint_positions_initial)
					self.moveit2.wait_until_executed()
					self.moveit2.move_to_configuration(joint_positions_final_1)
					self.moveit2.wait_until_executed()
					self.moveit2.move_to_configuration(joint_positions_final_1)
					self.moveit2.wait_until_executed()

					self.pz = self.pz +1
				elif self.pz == 1:
					self.moveit2.move_to_configuration(joint_positions_initial)
					self.moveit2.wait_until_executed()
					self.moveit2.move_to_configuration(joint_positions_final_2)
					self.moveit2.wait_until_executed()
					self.moveit2.move_to_configuration(joint_positions_final_2)
					self.moveit2.wait_until_executed()

					self.pz = self.pz +1
				else:
					self.moveit2.move_to_configuration(joint_positions_initial)
					self.moveit2.wait_until_executed()
					self.moveit2.move_to_configuration(joint_positions_final_3)
					self.moveit2.wait_until_executed()
					self.moveit2.move_to_configuration(joint_positions_final_3)
					self.moveit2.wait_until_executed()


				while not self.gripper_control.wait_for_service(timeout_sec=1.0):
					self.enftf.get_logger().info('EEF service not available, waiting again...')
				print("tryiing to attach")
				req = DetachLink.Request()
				req.model1_name =  f'box{box_no}'      
				req.link1_name  = 'link'       
				req.model2_name =  'ur5'       
				req.link2_name  = 'wrist_3_link'
				self.gripper_control_off.call_async(req)
				break
					
			except:
				pass


def main():
	rclpy.init()
	enftf = endf()


	twist_msg = TwistStamped()
	#twist_msg.header.frame_id = "auto"
	
	#node = Node("move_ur5")

	executor = rclpy.executors.MultiThreadedExecutor(2)
	executor.add_node(enftf)
	executor_thread = Thread(target=executor.spin, daemon=True, args=())
	executor_thread.start()

	filepath1 = RACK_MESH
	filepath2 = BASE_MESH
	position_r1 = [0.52,0.05,0.17]
	quat_xyzw_r1 = [0.00,0.00,0.00,0.00]
	position_r2 = [0.25,-0.62,0.17]
	quat_xyzw_r2 = [0, 0, 0.7068252, 0.7073883]
	position_r3 = [0.25,0.74,0.17]
	quat_xyzw_r3 = [0, 0, -0.7068252, 0.7073883]
	# position_b1 = [0.25,-0.57,0.18]
	# quat_xyzw_b1 = [0, 0, 0.7068252, 0.7073883]
	# position_b2 = [0.25,-0.57,0.55]
	# quat_xyzw_b2 = [-0.0000221, 0.001126, 0.6931103, 0.7208306]
	# position_b3 = [0.45,0.06,0.55]
	# quat_xyzw_b3 = [0, 0, 0.9999997, 0.0007963]
	rack_pos = [position_r1,position_r2,position_r3]
	rack_quat = [quat_xyzw_r1,quat_xyzw_r2,quat_xyzw_r3]
	
	pos_b = [-0.20, 0, -0.26]
	quat_b= [0, 0, 1, 0.000796327]
	base_pos=[pos_b,quat_b]
	# box_pos = [position_b1,position_b2,position_b3]
	# box_quat = [quat_xyzw_b1,quat_xyzw_b2,quat_xyzw_b3]
	#check if exists

	# if not path.exists(filepath1):
	#     node.get_logger().error(f"File '{filepath1}' does not exist")
	#     rclpy.shutdown()
	#     exit(1)
	# if not path.exists(filepath2):
	#     node.get_logger().error(f"File '{filepath2}' does not exist")
	#     rclpy.shutdown()
	#     exit(1)

	mesh_id1 = path.basename(filepath1).split(".")[0]
	mesh_id2 = path.basename(filepath2).split(".")[0]
	mesh_box_id = ["box_1","box_2","box_3"]
	mesh_rack_id = ["rack_1","rack_2","rack_3"]
	joint_positions_initial = (enftf.get_parameter("joint_positions_initial").get_parameter_value().double_array_value)
	joint_positions_positive = (enftf.get_parameter("joint_positions_positive").get_parameter_value().double_array_value)
	joint_positions_negative = (enftf.get_parameter("joint_positions_negative").get_parameter_value().double_array_value)


	for i in range(len(position_r1)):
			time.sleep(2)
			enftf.moveit2.add_collision_mesh(
				filepath=filepath1, id=mesh_rack_id[i], position=rack_pos[i], quat_xyzw=rack_quat[i], frame_id=ur5.base_link_name()
			)
			time.sleep(2)
			# print(filepath1)
			# moveit2.add_collision_mesh(
			#     filepath=filepath2, id=mesh_box_id[i], position=box_pos[i], quat_xyzw=box_quat[i], frame_id=ur5.base_link_name()
			# )
			# time.sleep(2)
	enftf.moveit2.add_collision_mesh(
	filepath=filepath2, id="base", position=base_pos[0], quat_xyzw=base_pos[1], frame_id=ur5.base_link_name())
	enftf.servo(str(1))

	enftf.moveit2.move_to_configuration(joint_positions_initial)
	enftf.moveit2.wait_until_executed()
	enftf.moveit2.move_to_configuration(joint_positions_positive)
	enftf.moveit2.wait_until_executed()
	enftf.servo(str(49))

	enftf.moveit2.move_to_configuration(joint_positions_initial)
	enftf.moveit2.wait_until_executed()
	enftf.moveit2.move_to_configuration(joint_positions_negative)
	enftf.moveit2.wait_until_executed()
	enftf.servo(str(3))





	

		

	


if __name__ == "__main__":
	main()
