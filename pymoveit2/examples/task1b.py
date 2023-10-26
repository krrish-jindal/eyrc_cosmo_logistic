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
BOX_MESH=path.join(
	path.dirname(path.realpath(__file__)), "assets", "box.stl"
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

def main():
	rclpy.init()
	enftf = endf()
	twist_pub = enftf.create_publisher(TwistStamped, "/servo_node/delta_twist_cmds", 10)
	gripper_control = enftf.create_client(AttachLink, '/GripperMagnetON')
	gripper_control_off = enftf.create_client(DetachLink, '/GripperMagnetOFF')

	twist_msg = TwistStamped()
	#twist_msg.header.frame_id = "auto"
	
	#node = Node("move_ur5")
	callback_group = ReentrantCallbackGroup()

	moveit2 = MoveIt2(
		node=enftf,
		joint_names=ur5.joint_names(),
		base_link_name=ur5.base_link_name(),
		end_effector_name=ur5.end_effector_name(),
		group_name=ur5.MOVE_GROUP_ARM,
		callback_group=callback_group,
	)

	executor = rclpy.executors.MultiThreadedExecutor(2)
	executor.add_node(enftf)
	executor_thread = Thread(target=executor.spin, daemon=True, args=())
	executor_thread.start()

	filepath1 = RACK_MESH
	filepath2 = BASE_MESH
	filepath3 = BOX_MESH
	position_r1 = [0.52,0.05,0.17]
	quat_xyzw_r1 = [0.00,0.00,0.00,0.00]
	position_r2 = [0.25,-0.62,0.17]
	quat_xyzw_r2 = [0, 0, 0.7068252, 0.7073883]
	position_r3 = [0.25,0.74,0.17]
	quat_xyzw_r3 = [0, 0, -0.7068252, 0.7073883]
	position_b1 = [-0.1,-0.57,0.55]
	quat_xyzw_b1 = [0, 0, 0.7068252, 0.7073883]
	position_b2 = [-0.05,0.65,0.55]
	quat_xyzw_b2 = [-0.0000221, 0.001126, 0.6931103, 0.7208306]
	
	position_b3 = [0.45,-0.14,0.55]
	quat_xyzw_b3 = [0, 0, 0.9999997, 0.0007963]
	

	position_b4 = [0.25,-0.57,0.55]
	quat_xyzw_b4 = [0, 0, 0.7068252, 0.7073883]
	position_b5 = [0.3,0.65,0.55]
	quat_xyzw_b5 = [-0.0000221, 0.001126, 0.6931103, 0.7208306]
	
	position_b6 = [0.45,0.34,0.55]
	quat_xyzw_b6 = [0, 0, 0.9999997, 0.0007963]

	pos_b = [-0.20, 0, -0.26]
	quat_b= [0, 0, 1, 0.000796327]

	rack_pos = [position_r1,position_r2,position_r3]
	rack_quat = [quat_xyzw_r1,quat_xyzw_r2,quat_xyzw_r3]
	base_pos=[pos_b,quat_b]

	box_pos = [position_b1,position_b2,position_b3]
	box_quat = [quat_xyzw_b1,quat_xyzw_b2,quat_xyzw_b3]

	box_pos_2 = [position_b4,position_b5,position_b6]
	box_quat_2 = [quat_xyzw_b4,quat_xyzw_b5,quat_xyzw_b6]

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
	mesh_box_id_2 = ["box_6","box_4","box_5"]

	mesh_rack_id = ["rack_1","rack_2","rack_3"]
	__twist_pub = enftf.create_publisher(TwistStamped, "/servo_node/delta_twist_cmds", 10)



	for i in range(len(position_r1)):
			moveit2.add_collision_mesh(
				filepath=filepath1, id=mesh_rack_id[i], position=rack_pos[i], quat_xyzw=rack_quat[i], frame_id=ur5.base_link_name()
			)
			time.sleep(2)
			# print(filepath1)
			moveit2.add_collision_mesh(
			    filepath=filepath3, id=mesh_box_id[i], position=box_pos[i], quat_xyzw=box_quat[i], frame_id=ur5.base_link_name()
			)
			time.sleep(2)
			moveit2.add_collision_mesh(
			    filepath=filepath3, id=mesh_box_id_2[i], position=box_pos_2[i], quat_xyzw=box_quat_2[i], frame_id=ur5.base_link_name()
			)

			time.sleep(2)
	moveit2.add_collision_mesh(
	filepath=filepath2, id="base", position=base_pos[0], quat_xyzw=base_pos[1], frame_id=ur5.base_link_name())
	time.sleep(2)

	while rclpy.ok():
		try:
			tool0 = enftf.tf_buffer.lookup_transform('base_link', "tool0", rclpy.time.Time())
			box1 = enftf.tf_buffer.lookup_transform('base_link', "obj_1", rclpy.time.Time())

			while rclpy.ok():
				try:
					box1 = enftf.tf_buffer.lookup_transform('base_link', "obj_1", rclpy.time.Time())
					tool0 = enftf.tf_buffer.lookup_transform('base_link', "tool0", rclpy.time.Time())
					# print(round(tool0.transform.translation.x, 2))
					# print(round(box1.transform.translation.x, 2))
				except:
					pass
				x=round((box1.transform.translation.y) - (tool0.transform.translation.y),4)
				y=round((box1.transform.translation.x) - (tool0.transform.translation.x),4)
				z=round((box1.transform.translation.z) - (tool0.transform.translation.z),4)
				print("----------")
				moveit2.move_to_pose(position=[box1.transform.translation.x,box1.transform.translation.y,box1.transform.translation.z], quat_xyzw=[box1.transform.rotation.x, box1.transform.rotation.y, box1.transform.rotation.z, box1.transform.rotation.w], cartesian=True,tolerance_position = 0.01,tolerance_orientation=0.01)
				moveit2.wait_until_executed()

				print("+++++++++++++")
				while not gripper_control.wait_for_service(timeout_sec=1.0):
					enftf.get_logger().info('EEF service not available, waiting again...')
				print("tryiing to attach")
				req = AttachLink.Request()
				req.model1_name =  'box1'      
				req.link1_name  = 'link'       
				req.model2_name =  'ur5'       
				req.link2_name  = 'wrist_3_link'
				gripper_control.call_async(req)
				time.sleep(2)
				while not gripper_control.wait_for_service(timeout_sec=1.0):
					enftf.get_logger().info('EEF service not available, waiting again...')
				print("tryiing to attach")
				req = AttachLink.Request()
				req.model1_name =  'box1'      
				req.link1_name  = 'link'       
				req.model2_name =  'ur5'       
				req.link2_name  = 'wrist_3_link'
				gripper_control.call_async(req)
				time.sleep(2)
				break

			while rclpy.ok():
				try:
					box1 = enftf.tf_buffer.lookup_transform('base_link', "obj_1", rclpy.time.Time())
					tool0 = enftf.tf_buffer.lookup_transform('base_link', "tool0", rclpy.time.Time())
				except:
					pass
				print("+-+-0",round(box1.transform.rotation.z,2))
				print(round(tool0.transform.translation.x,2))
				if 0.6>round(box1.transform.rotation.z,2)>0.40:

					if (round(tool0.transform.translation.x,2) > 0.24):
						print("----1--------")
						__twist_msg = TwistStamped()
						__twist_msg.header.stamp = enftf.get_clock().now().to_msg()
						__twist_msg.header.frame_id = ur5.base_link_name()
						__twist_msg.twist.linear.z = 0.1
						__twist_msg.twist.linear.x = -0.4
						__twist_pub.publish(__twist_msg)
					else:
						break

				elif round(box1.transform.rotation.z,2)<= 0.0:
					print("$$$",tool0.transform.translation.y)
					if (round(tool0.transform.translation.y,2) < -0.21):
						print("----2--------")
						
						__twist_msg = TwistStamped()
						__twist_msg.header.stamp = enftf.get_clock().now().to_msg()
						__twist_msg.header.frame_id = ur5.base_link_name()
						__twist_msg.twist.linear.z = 0.1
						__twist_msg.twist.linear.y = 0.2

						__twist_pub.publish(__twist_msg)
					else:
						break

				elif round(box1.transform.rotation.z,2)> 0.6:
					if (round(tool0.transform.translation.y,2) < 0.21):
						print("----3--------")
						
						__twist_msg = TwistStamped()
						__twist_msg.header.stamp = enftf.get_clock().now().to_msg()
						__twist_msg.header.frame_id = ur5.base_link_name()
						__twist_msg.twist.linear.z = 0.1
						__twist_msg.twist.linear.y = -0.2

						__twist_pub.publish(__twist_msg)
					else:
						break
				else:
					break				
			print("***********")
			# moveit2.move_to_configuration(joint_positions=[0.0,-2.39,2.4,-3.15,-1.58,3.15], joint_names=["shoulder_pan_joint",  "shoulder_lift_joint",  "elbow_joint",  "wrist_1_joint", "wrist_2_joint",  "wrist_3_joint"] ,tolerance_position = 0.3,tolerance_orientation=0.3)
			# moveit2.wait_until_executed()
			

			
			moveit2.move_to_pose(position=[-0.8, 0.12, 0.5], quat_xyzw=[-box1.transform.rotation.x, box1.transform.rotation.y, -box1.transform.rotation.z, -box1.transform.rotation.w], cartesian=True,tolerance_position = 0.3,tolerance_orientation=0.3)
			moveit2.wait_until_executed()
			enftf.switch == True
			print("***********")
			moveit2.move_to_pose(position=[-0.8, 0.12, 0.5], quat_xyzw=[-box1.transform.rotation.x, box1.transform.rotation.y, -box1.transform.rotation.z, -box1.transform.rotation.w], cartesian=True,tolerance_position = 0.3,tolerance_orientation=0.3)
			moveit2.wait_until_executed()
			enftf.switch == True

			while not gripper_control.wait_for_service(timeout_sec=1.0):
				enftf.get_logger().info('EEF service not available, waiting again...')
			print("tryiing to Dettach")
			req = DetachLink.Request()
			req.model1_name =  'box1'      
			req.link1_name  = 'link'       
			req.model2_name =  'ur5'       
			req.link2_name  = 'wrist_3_link'
			gripper_control_off.call_async(req)

			break

				# else:
				#     rclpy.shutdown()
				#     exit(0)

				# if round((box1.transform.translation.x) - (tool0.transform.translation.x),4) > 0.0002:
				#     __twist_msg = TwistStamped()
				#     __twist_msg.header.stamp = enftf.get_clock().now().to_msg()
				#     __twist_msg.header.frame_id = ur5.base_link_name()
				#     __twist_msg.twist.linear.x = 0.2
				#     __twist_pub.publish(__twist_msg)
				# elif round((box1.transform.translation.x) - (tool0.transform.translation.x),4) < 0.1:
				#     __twist_msg = TwistStamped()
				#     __twist_msg.header.stamp = enftf.get_clock().now().to_msg()
				#     __twist_msg.header.frame_id = ur5.base_link_name()
				#     __twist_msg.twist.linear.x = -0.2
				#     __twist_pub.publish(__twist_msg)
				# if round((box1.transform.translation.z) - (tool0.transform.translation.z),4) > 0.1:
				#     __twist_msg = TwistStamped()
				#     __twist_msg.header.stamp = enftf.get_clock().now().to_msg()
				#     __twist_msg.header.frame_id = ur5.base_link_name()
				#     __twist_msg.twist.linear.z = 0.2
				#     __twist_pub.publish(__twist_msg)
				# elif round((box1.transform.translation.z) - (tool0.transform.translation.z),4) < 0.1:
				#     __twist_msg = TwistStamped()
				#     __twist_msg.header.stamp = enftf.get_clock().now().to_msg()
				#     __twist_msg.header.frame_id = ur5.base_link_name()
				#     __twist_msg.twist.linear.z = -0.2
				#     __twist_pub.publish(__twist_msg)

				

			
			# while round(tool0.transform.translation.x,2) > -1.20:
			#     try:
			#         tool0 = enftf.tf_buffer.lookup_transform('base_link', "tool0", rclpy.time.Time())
			#         print((tool0.transform.translation.x))
			#     except:
			#         pass
			#     __twist_msg = TwistStamped()
			#     __twist_msg.header.stamp = enftf.get_clock().now().to_msg()
			#     __twist_msg.header.frame_id = ur5.base_link_name()
			#     __twist_msg.twist.linear.x = -0.5
			#     __twist_pub.publish(__twist_msg)
			# __twist_msg_stop = TwistStamped()
			# __twist_msg_stop.header.stamp = enftf.get_clock().now().to_msg()
			# __twist_msg_stop.header.frame_id = ur5.base_link_name()
			# __twist_msg_stop.twist.linear.x = 0.0
			# __twist_pub.publish(__twist_msg_stop)



	

			
		except:
			pass

		

	rclpy.shutdown()
	exit(0)


if __name__ == "__main__":
	main()
