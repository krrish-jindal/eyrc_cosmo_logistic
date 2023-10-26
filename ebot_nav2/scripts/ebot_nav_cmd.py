#! /usr/bin/env python3
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from ebot_docking.srv import DockSw


def send_request(orientation):
        request_dock = DockSw.Request()
        request_dock.orientation = orientation
        future = client_docking.call_async(request_dock)
        rclpy.spin_until_future_complete(node, future)
        return future.result()

def main():
    global client_docking, node
    rclpy.init()

    node = rclpy.create_node("nav_dock")

    client_docking = node.create_client(srv_type=DockSw, srv_name='dock_control')

    navigator = BasicNavigator()
    
    orientation_rack_1 = 3.139999
    orientation_rack_2 = -1.570000
    orientation_rack_3 = 1.569999

    goal_pick_1 = PoseStamped()
    goal_pick_1.header.frame_id = 'map'
    goal_pick_1.header.stamp = navigator.get_clock().now().to_msg()
    goal_pick_1.pose.position.x = 0.32509
    goal_pick_1.pose.position.y = 4.385162
    goal_pick_1.pose.orientation.x = 0.0
    goal_pick_1.pose.orientation.y = 0.0
    goal_pick_1.pose.orientation.z = 0.7077099
    goal_pick_1.pose.orientation.w = 0.7065031


    goal_pick_2 = PoseStamped()
    goal_pick_2.header.frame_id = 'map'
    goal_pick_2.pose.position.x = 1.949751
    goal_pick_2.pose.position.y = 2.108228
    goal_pick_2.pose.orientation.x = 0.0
    goal_pick_2.pose.orientation.y = 0.0
    goal_pick_2.pose.orientation.z = 0.0
    goal_pick_2.pose.orientation.w = 1.0

    goal_pick_3 = PoseStamped()
    goal_pick_3.header.frame_id = 'map'
    goal_pick_3.header.stamp = navigator.get_clock().now().to_msg()
    goal_pick_3.pose.position.x = 1.964522
    goal_pick_3.pose.position.y = -6.966175
    goal_pick_3.pose.orientation.x = 0.0
    goal_pick_3.pose.orientation.y = 0.0
    goal_pick_3.pose.orientation.z = 0.0
    goal_pick_3.pose.orientation.w = 0.0

    navigator.waitUntilNav2Active()

    navigator.goToPose(goal_pick_1)

    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()
        if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
            navigator.cancelTask()

    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Goal1 succeeded!')
    elif result == TaskResult.CANCELED:
        print('Goal was canceled!')
    elif result == TaskResult.FAILED:
        print('Goal failed!')
    else:
        print('Goal has an invalid return status!')
    
    send_request(orientation_rack_1)
    
    navigator.goToPose(goal_pick_2)

    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()
      
        if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
            navigator.cancelTask()
    
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Goal2 succeeded!')
    elif result == TaskResult.CANCELED:
        print('Goal was canceled!')
    elif result == TaskResult.FAILED:
        print('Goal failed!')
    else:
        print('Goal has an invalid return status!')

    send_request(orientation_rack_2)

    
    navigator.goToPose(goal_pick_3)

    
    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()
  
        if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
            navigator.cancelTask()

    # Do something depending on the return code
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Goal3 succeeded!')
    elif result == TaskResult.CANCELED:
        print('Goal was canceled!')
    elif result == TaskResult.FAILED:
        print('Goal failed!')
    else:
        print('Goal has an invalid return status!')

    send_request(orientation_rack_3)

    navigator.lifecycleShutdown()

    exit(0)


if __name__ == '__main__':
    main()