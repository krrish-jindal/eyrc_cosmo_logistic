#! /usr/bin/env python3
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
import os 
from rclpy.duration import Duration
from rclpy.node import Node
from ebot_docking.srv import DockSw
from linkattacher_msgs.srv import AttachLink
from linkattacher_msgs.srv import DetachLink
import yaml
from geometry_msgs.msg import Twist
import time
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory




def send_request(orientation):

        request_dock = DockSw.Request()
        request_dock.orientation_dock = True
        request_dock.orientation = orientation
        future = client_docking.call_async(request_dock)
        rclpy.spin_until_future_complete(node, future)
        return future.result()

def rack_attach(rack):

    req = AttachLink.Request()
    req.model1_name =  'ebot'      
    req.link1_name  = 'ebot_base_link'       
    req.model2_name =  rack       
    req.link2_name  = 'link'

    atc=attach.call_async(req)
    rclpy.spin_until_future_complete(node, atc)
    vel_msg.linear.x=0.2
    vel_pub.publish(vel_msg)
    time.sleep(2)
    vel_msg.linear.x=0.0
    vel_pub.publish(vel_msg)
    return atc.result()

def nav_reach(goal):

    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()
      
        if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
            navigator.cancelTask()
    
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print(f'Goal{str(goal)} succeeded!')
    elif result == TaskResult.CANCELED:
        print('Goal was canceled!')
    elif result == TaskResult.FAILED:
        print('Goal failed!')
    else:
        print('Goal has an invalid return status!')

def rack_detach(rack):

    req = DetachLink.Request()
    req.model1_name =  'ebot'      
    req.link1_name  = 'ebot_base_link'       
    req.model2_name =  rack       
    req.link2_name  = 'link'

    dtc=detach.call_async(req)
    rclpy.spin_until_future_complete(node, dtc)
    vel_msg.linear.x=0.2
    vel_pub.publish(vel_msg)
    time.sleep(2)
    vel_msg.linear.x=0.0
    vel_pub.publish(vel_msg)
    return dtc.result()


def main():
    package_name = 'ebot_nav2'
    config="config/config.yaml"

    global client_docking, node ,attach ,detach, navigator ,vel_msg,vel_pub
    rclpy.init()
    ebot_nav2_dir = get_package_share_directory('ebot_nav2')

    pkg_share = FindPackageShare(package=package_name).find(package_name)
    config_path = os.path.join(pkg_share, config)
    with open(config_path, 'r') as infp:
        pos_rack = infp.read()

    data_dict = yaml.safe_load(pos_rack)

    # Access elements of the dictionary
    positions = data_dict['position']
    rack1_coordinates = positions[0]['rack1']
    package_id = data_dict['package_id'][0]
    print("CONFIG_PATH",package_id)
   

    node = rclpy.create_node("nav_dock")
    attach = node.create_client(srv_type=AttachLink, srv_name='/ATTACH_LINK')

    detach = node.create_client(srv_type=DetachLink, srv_name='/DETACH_LINK')


    client_docking = node.create_client(srv_type=DockSw, srv_name='dock_control')
    vel_pub = node.create_publisher(Twist, "/cmd_vel", 10)
    vel_msg=Twist()
    navigator = BasicNavigator()
    
    orientation_rack_1 = 3.139999
    orientation_rack_2 = -1.570000
    orientation_rack_3 = 1.569999
    rack_list=["rack1","Rack2","Rack3"]

    goal_pick_1 = PoseStamped()
    goal_pick_1.header.frame_id = 'map'
    goal_pick_1.header.stamp = navigator.get_clock().now().to_msg()
    goal_pick_1.pose.position.x = 0.108200
    goal_pick_1.pose.position.y = 4.528524
    goal_pick_1.pose.orientation.x = 0.0
    goal_pick_1.pose.orientation.y = 0.0
    goal_pick_1.pose.orientation.z = 0.7077099
    goal_pick_1.pose.orientation.w = 0.7065031


    goal_pick_2 = PoseStamped()
    goal_pick_2.header.frame_id = 'map'
    goal_pick_2.pose.position.x = 1.960219
    goal_pick_2.pose.position.y = 2.118804
    goal_pick_2.pose.orientation.x = 0.0
    goal_pick_2.pose.orientation.y = 0.0
    goal_pick_2.pose.orientation.z = 0.0
    goal_pick_2.pose.orientation.w = 1.0

    goal_pick_3 = PoseStamped()
    goal_pick_3.header.frame_id = 'map'
    goal_pick_3.header.stamp = navigator.get_clock().now().to_msg()
    goal_pick_3.pose.position.x = 1.998759
    goal_pick_3.pose.position.y = -7.102119
    goal_pick_3.pose.orientation.x = 0.0
    goal_pick_3.pose.orientation.y = 0.0
    goal_pick_3.pose.orientation.z = 0.0
    goal_pick_3.pose.orientation.w = 1.0

    goal_drop_1 = PoseStamped()
    goal_drop_1.header.frame_id = 'map'
    goal_drop_1.header.stamp = navigator.get_clock().now().to_msg()
    goal_drop_1.pose.position.x = 0.5
    goal_drop_1.pose.position.y = -2.455
    goal_drop_1.pose.orientation.x = 0.0
    goal_drop_1.pose.orientation.y = 0.0
    goal_drop_1.pose.orientation.z = 0.9999997
    goal_drop_1.pose.orientation.w =  0.0007963 

    goal_drop_2 = PoseStamped()
    goal_drop_2.header.frame_id = 'map'
    goal_drop_2.header.stamp = navigator.get_clock().now().to_msg()
    goal_drop_2.pose.position.x = 1.650000
    goal_drop_2.pose.position.y = -3.684832
    goal_drop_2.pose.orientation.x = 0.0
    goal_drop_2.pose.orientation.y = 0.0
    goal_drop_2.pose.orientation.z =  -0.70398
    goal_drop_2.pose.orientation.w =  0.7102198

    goal_drop_3 = PoseStamped()
    goal_drop_3.header.frame_id = 'map'
    goal_drop_3.header.stamp = navigator.get_clock().now().to_msg()
    goal_drop_3.pose.position.x =1.550000
    goal_drop_3.pose.position.y = -1.298586
    goal_drop_3.pose.orientation.x = 0.0
    goal_drop_3.pose.orientation.y = 0.0
    goal_drop_3.pose.orientation.z =  0.6921004
    goal_drop_3.pose.orientation.w = 0.7218012 
    navigator.waitUntilNav2Active()
    print("POSS",f"goal_pick_{package_id}")
# Pick Rack_1
    if package_id ==3:
        navigator.goToPose(goal_pick_3)
        nav_reach(3)
        send_request(orientation_rack_3)
        rack_attach(rack_list[2])
    elif package_id ==2:
        navigator.goToPose(goal_pick_2)
        nav_reach(2)
        send_request(orientation_rack_2)
        rack_attach(rack_list[1])
    else:
        navigator.goToPose(goal_pick_1)
        nav_reach(1)
        send_request(orientation_rack_1)
        rack_attach(rack_list[0])

# Drop Rack_1
    navigator.goToPose(goal_drop_1)
    nav_reach(2)
    rack_detach(rack_list[0])

# Pick Rack_2
    # navigator.goToPose(goal_pick_2)
    # nav_reach(3)
    # send_request(orientation_rack_2)
    # rack_attach(rack_list[1])

#  Drop Rack_2
#     navigator.goToPose(goal_drop_2)
#     nav_reach(4)
#     rack_detach(rack_list[1])


# # Pick Rack_3
#     navigator.goToPose(goal_pick_3)
#     nav_reach(5)
#     send_request(orientation_rack_3)
#     rack_attach(rack_list[2])
    
# #  Drop Rack_3
#     navigator.goToPose(goal_drop_3)
#     nav_reach(6)
#     rack_detach(rack_list[2])
    

    navigator.lifecycleShutdown()

    exit(0)


if __name__ == '__main__':
    main()