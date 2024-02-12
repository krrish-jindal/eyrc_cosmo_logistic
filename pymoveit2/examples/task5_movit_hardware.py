#!/usr/bin/env python3

from os import path
from threading import Thread

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
import time
from pymoveit2 import MoveIt2
from pymoveit2.robots import ur5
from arm_picky.srv import ArmNew
import math
import sys
import tf2_ros
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from scipy.spatial.transform import Rotation as R
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Quaternion
from tf_transformations import quaternion_from_euler
from tf_transformations import euler_from_quaternion
import tf_transformations
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TwistStamped

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

from ur_msgs.srv import SetIO
from controller_manager_msgs.srv import SwitchController # module call

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
        self.callback_group = ReentrantCallbackGroup()
        self.tf_buffer = tf2_ros.buffer.Buffer()                                        # buffer time used for listening transforms
        self.listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.br = tf2_ros.TransformBroadcaster(self)
        self.switch = False
        self.arm_srv = self.create_service(ArmNew, 'arm_control', self.arm_control_callback, callback_group=self.callback_group)
        self.__contolMSwitch = self.create_client(SwitchController, "/controller_manager/switch_controller")
        self.tf_topic = self.create_subscription(TFMessage, '/tf', self.tf_cb , 10)
        self.twist_pub = self.create_publisher(TwistStamped, "/servo_node/delta_twist_cmds", 10)
        self.obj_aruco = "None"
        self.last_obj = "None"

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
            "joint_positions_back",
            [
                -2.96706,
                -1.6057,
                1.67552,
                -3.14159,
                -1.58825,
                3.14159
            ],
        )
        self.declare_parameter(
        "joint_positions_final_1",
        [
            -0.027,
            -1.803,
            -1.3658,
            -3.039,
            -1.52,
            3.15
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
            -2.967,
            -0.715,
            0.89,
            -3.14159,
            -1.58825,
            3.14159
        ],
    )

    def arm_control_callback(self, request, response):
        print("I am here")
        self.variable_a = request.boom  
        self.variable_b = request.rack_no
        response.reply = True
        return response
    
    def trajactory(self):
        switchParam = SwitchController.Request()
        switchParam.activate_controllers = ["scaled_joint_trajectory_controller"]# for normal use of moveit
        switchParam.deactivate_controllers = ["forward_position_controller"]
        switchParam.strictness = 2
        switchParam.start_asap = False

        while not self.__contolMSwitch.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn(f"Service control Manager is not yet available...")
        self.__contolMSwitch.call_async(switchParam)
        print("[CM]: Switching Complete Trajec")

    def servo_active(self):
        switchParam = SwitchController.Request()
        print("hello ")
        switchParam.deactivate_controllers = ["scaled_joint_trajectory_controller"]
        switchParam.activate_controllers = ["forward_position_controller"] # for servoing
        switchParam.strictness = 2
        switchParam.start_asap = False

        while not self.__contolMSwitch.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn(f"Service control Manager is not yet available...")
        self.__contolMSwitch.call_async(switchParam)
        print("[CM]: Switching Complete Servo")

        while not self.__contolMSwitch.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn(f"Service control Manager is not yet available...")
        self.__contolMSwitch.call_async(switchParam)
        print("[CM]: Switching Complete")

    def gripper_call(self, state):
        '''
        based on the state given as i/p the service is called to activate/deactivate
        pin 16 of TCP in UR5
        i/p: node, state of pin:Bool
        o/p or return: response from service call
        '''
        gripper_control = self.create_client(SetIO, '/io_and_status_controller/set_io')
        while not gripper_control.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('EEF Tool service not available, waiting again...')
        req         = SetIO.Request()
        req.fun     = 1
        req.pin     = 16
        req.state   = float(state)
        gripper_control.call_async(req)
        return state
    
    def tf_cb(self, data):
        self.obj = "None"
        try:
            if data.transforms[0].header.frame_id == "base_link":
                if "obj" in data.transforms[0].child_frame_id:
                    self.obj_aruco = data.transforms[0].child_frame_id
        except Exception as e:
            pass

    def servo(self, box_no):
        while rclpy.ok():
            try:
                tool0 = self.tf_buffer.lookup_transform('base_link', "tool0", rclpy.time.Time())
                box49 = self.tf_buffer.lookup_transform('base_link', f"obj_{box_no}", rclpy.time.Time())

                while rclpy.ok():
                    try:
                        box49 = self.tf_buffer.lookup_transform('base_link', f"obj_{box_no}", rclpy.time.Time())
                        tool0 = self.tf_buffer.lookup_transform('base_link', "tool0", rclpy.time.Time())
                    except Exception as e:
                        pass
                    print(f"y ={round((box49.transform.translation.y) - (tool0.transform.translation.y),4)}")
                    print(f"x ={round((box49.transform.translation.x) - (tool0.transform.translation.x),4)}")
                    print(f"z ={round((box49.transform.translation.z) - (tool0.transform.translation.z),4)}")
                    joint_positions_initial = (self.get_parameter("joint_positions_initial").get_parameter_value().double_array_value)
                    joint_positions_final_1 = (self.get_parameter("joint_positions_final_1").get_parameter_value().double_array_value)
                    joint_positions_final_2 = (self.get_parameter("joint_positions_final_2").get_parameter_value().double_array_value)
                    joint_positions_final_3 = (self.get_parameter("joint_positions_final_3").get_parameter_value().double_array_value)
                    joint_positions_back = (self.get_parameter("joint_positions_back").get_parameter_value().double_array_value)


 #  ARM Move To BOX 

                    if round((box49.transform.translation.y) - (tool0.transform.translation.y),4) > 0.004 or round((box49.transform.translation.x) - (tool0.transform.translation.x),4) > 0.004 or round((box49.transform.translation.z) - (tool0.transform.translation.z),4) > 0.004:
                        __twist_msg = TwistStamped()
                        __twist_msg.header.stamp = self.get_clock().now().to_msg()
                        __twist_msg.header.frame_id = ur5.base_link_name()
                        __twist_msg.twist.linear.y = round((box49.transform.translation.y) - (tool0.transform.translation.y),4) *2
                        __twist_msg.twist.linear.x = round((box49.transform.translation.x) - (tool0.transform.translation.x),4) *2
                        __twist_msg.twist.linear.z = round((box49.transform.translation.z) - (tool0.transform.translation.z),4) *2
                        self.twist_pub.publish(__twist_msg)
                        print("SERVO_START")

#   Gripper ON
                    else :
                        print("trying to attach")
                        self.gripper_call(1)
                        print("DONE")
                        break

                while rclpy.ok():
                    try:
                        box49 = self.tf_buffer.lookup_transform('base_link', f"obj_{box_no}", rclpy.time.Time())
                        tool0 = self.tf_buffer.lookup_transform('base_link', "tool0", rclpy.time.Time())
                        roll , pitch , yaw  = euler_from_quaternion([box49.transform.rotation.x, box49.transform.rotation.y, box49.transform.rotation.z, box49.transform.rotation.w])


                    except Exception as e:
                        pass


#   ARM Adding Buffer POSS  &  Drop Location
                        



#  LEFT RACK  


                    if self.variable_b == "3" and (round(tool0.transform.translation.y,2) > 0.25):
                        __twist_msg = TwistStamped()
                        __twist_msg.header.stamp = self.get_clock().now().to_msg()
                        __twist_msg.header.frame_id = ur5.base_link_name()
                        __twist_msg.twist.linear.z = 0.1
                        __twist_msg.twist.linear.y = -0.2
                        self.twist_pub.publish(__twist_msg)
                    elif self.variable_b == "3" and (round(tool0.transform.translation.y,2) <= 0.25):
                        self.trajactory()
                        self.moveit2.move_to_configuration(joint_positions_initial)
                        self.moveit2.wait_until_executed()
                        self.moveit2.move_to_configuration(joint_positions_initial)
                        self.moveit2.wait_until_executed()
                        self.moveit2.move_to_configuration(joint_positions_final_2)
                        self.moveit2.wait_until_executed()
                        self.moveit2.move_to_configuration(joint_positions_final_2)
                        self.moveit2.wait_until_executed()
                        break 


#  FRONT RACK
                    
                    elif self.variable_b == "2" and (round(tool0.transform.translation.x,2) > 0.23):
                        __twist_msg = TwistStamped()
                        __twist_msg.header.stamp = self.get_clock().now().to_msg()
                        __twist_msg.header.frame_id = ur5.base_link_name()
                        __twist_msg.twist.linear.z = 0.1
                        __twist_msg.twist.linear.x = -0.4
                        self.twist_pub.publish(__twist_msg)
                    elif self.variable_b == "2" and (round(tool0.transform.translation.x,2) <= 0.23):
                        self.trajactory()
                        self.moveit2.move_to_configuration(joint_positions_final_1)
                        self.moveit2.wait_until_executed()
                        self.moveit2.move_to_configuration(joint_positions_final_1)
                        self.moveit2.wait_until_executed()
                        break


 #  RIGHT RACK 

                    elif self.variable_b == "1" and (round(tool0.transform.translation.y,2) < -0.26):
                        __twist_msg = TwistStamped()
                        __twist_msg.header.stamp = self.get_clock().now().to_msg()
                        __twist_msg.header.frame_id = ur5.base_link_name()
                        __twist_msg.twist.linear.z = 0.4
                        __twist_msg.twist.linear.x = -0.4
                        self.twist_pub.publish(__twist_msg)
                    elif self.variable_b == "1" and (round(tool0.transform.translation.y,2) >= -0.26):
                        self.trajactory()
                        self.moveit2.move_to_configuration(joint_positions_back)
                        self.moveit2.wait_until_executed()
                        self.moveit2.move_to_configuration(joint_positions_back)
                        self.moveit2.wait_until_executed()
                        self.moveit2.move_to_configuration(joint_positions_final_3)
                        self.moveit2.wait_until_executed()
                        self.moveit2.move_to_configuration(joint_positions_final_3)
                        self.moveit2.wait_until_executed()
                        break



                   



#    Gripper OFF

                print("trying to detach")
                self.gripper_call(0)
                break
                    
            except Exception as e:
                print("Servo Function 2nd Part")
                print(e)


def main():

    rclpy.init()
    enftf = endf()

    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(enftf)
    executor_thread = Thread(target=executor.spin, daemon=True, args=())
    executor_thread.start()

    joint_positions_initial = (enftf.get_parameter("joint_positions_initial").get_parameter_value().double_array_value)
    joint_positions_positive = (enftf.get_parameter("joint_positions_positive").get_parameter_value().double_array_value)
    joint_positions_negative = (enftf.get_parameter("joint_positions_negative").get_parameter_value().double_array_value)
    joint_positions_back = (enftf.get_parameter("joint_positions_back").get_parameter_value().double_array_value)



    while rclpy.ok():
        try:
            obj = enftf.obj_aruco[4:]
            print(obj)
            box = enftf.tf_buffer.lookup_transform('base_link', enftf.obj_aruco, rclpy.time.Time())
            roll , pitch , yaw  = euler_from_quaternion([box.transform.rotation.x, box.transform.rotation.y, box.transform.rotation.z, box.transform.rotation.w])
            print(round(yaw,2))
            if enftf.variable_a == True:

                print("You are my special")

                if enftf.variable_b == "3":
                    enftf.trajactory()
                    enftf.moveit2.move_to_configuration(joint_positions_negative)
                    enftf.moveit2.wait_until_executed()
                    enftf.servo_active()
                    enftf.servo(obj)
                    enftf.trajactory()
                    enftf.moveit2.move_to_configuration(joint_positions_back)
                    enftf.moveit2.wait_until_executed()
                    enftf.moveit2.move_to_configuration(joint_positions_initial)
                    enftf.moveit2.wait_until_executed()
                    enftf.variable_a == False

                elif enftf.variable_b == "1":
                    enftf.trajactory()
                    enftf.moveit2.move_to_configuration(joint_positions_positive)
                    enftf.moveit2.wait_until_executed()
                    enftf.servo_active()
                    enftf.servo(obj)
                    enftf.trajactory()
                    enftf.moveit2.move_to_configuration(joint_positions_initial)
                    enftf.moveit2.wait_until_executed()
                    enftf.variable_a == False

                elif enftf.variable_b == "2":
                    enftf.servo_active()
                    enftf.servo(obj)
                    enftf.trajactory()
                    enftf.moveit2.move_to_configuration(joint_positions_initial)
                    enftf.moveit2.wait_until_executed()
                    enftf.variable_a == False

        except Exception as e:
            print("Main Function")

if __name__ == "__main__":
	main()