#!/usr/bin/env python3

from os import path
from threading import Thread

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

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
RACK_MESH = path.join(
    path.dirname(path.realpath(__file__)), "assets", "rack1.stl"
)
BOX_MESH = path.join(
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
    
    def start(self):
        try:
            t = self.tf_buffer.lookup_transform('base_link', "tool0", rclpy.time.Time())
            #print(t.transform.translation)
            self.endf_x = t.transform.translation.x
            self.endf_y = t.transform.translation.y
            self.endf_z = t.transform.translation.z
        except:
            pass

def main():
    rclpy.init()
    enftf = endf()
    twist_pub = enftf.create_publisher(TwistStamped, "/servo_node/delta_twist_cmds", 10)
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
    filepath2 = BOX_MESH
    position_r1 = [0.52,0.05,0.17]
    quat_xyzw_r1 = [0.00,0.00,0.00,0.00]
    position_r2 = [0.25,-0.62,0.17]
    quat_xyzw_r2 = [0, 0, 0.7068252, 0.7073883]
    position_r3 = [0.25,0.74,0.17]
    quat_xyzw_r3 = [0, 0, -0.7068252, 0.7073883]
    position_b1 = [0.25,-0.52,0.18]
    quat_xyzw_b1 = [0, 0, 0.7068252, 0.7073883]
    position_b2 = [0.25,-0.52,0.55]
    quat_xyzw_b2 = [-0.0000221, 0.001126, 0.6931103, 0.7208306]
    position_b3 = [0.45,0.06,0.55]
    quat_xyzw_b3 = [0, 0, 0.9999997, 0.0007963 ]
    rack_pos = [position_r1,position_r2,position_r3]
    rack_quat = [quat_xyzw_r1,quat_xyzw_r2,quat_xyzw_r3]
    box_pos = [position_b1,position_b2,position_b3]
    box_quat = [quat_xyzw_b1,quat_xyzw_b2,quat_xyzw_b3]
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
    twist_msg.twist.linear.z = 0.1
    twist_pub.publish(twist_msg)
    for i in range(len(position_r1)):
        moveit2.add_collision_mesh(
            filepath=filepath1, id=mesh_rack_id[i], position=rack_pos[i], quat_xyzw=rack_quat[i], frame_id=ur5.base_link_name()
        )
        print(filepath1)
        moveit2.add_collision_mesh(
            filepath=filepath2, id=mesh_box_id[i], position=box_pos[i], quat_xyzw=box_quat[i], frame_id=ur5.base_link_name()
        )

    while True:
        twist_msg.header.stamp = enftf.get_clock().now().to_msg()
        enftf.start()
        #print(truncate(enftf.endf_z,2))
        twist_pub.publish(twist_msg)
        if float(pos1[2]) > float(round(endf_z,2)) :
            twist_msg.twist.linear.x = 0.1
            twist_msg.twist.linear.z = 0.0
    
    rclpy.shutdown()
    exit(0)


if __name__ == "__main__":
    main()
