#! /usr/bin/env python3
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration


def main():
    rclpy.init()

    navigator = BasicNavigator()

    goal_pose1 = PoseStamped()
    goal_pose1.header.frame_id = 'map'
    goal_pose1.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose1.pose.position.x = 1.8
    goal_pose1.pose.position.y = 1.5
    goal_pose1.pose.orientation.x = 0.0
    goal_pose1.pose.orientation.y = 0.0
    goal_pose1.pose.orientation.z = 0.7068252
    goal_pose1.pose.orientation.w = 0.7073883


    goal_pose2 = PoseStamped()
    goal_pose2.header.frame_id = 'map'
    goal_pose2.pose.position.x = 2.0
    goal_pose2.pose.position.y = -7.0
    goal_pose1.pose.orientation.x = 0.0
    goal_pose1.pose.orientation.y = 0.0
    goal_pose1.pose.orientation.z = -0.7068252
    goal_pose1.pose.orientation.w = 0.7073883

    goal_pose3 = PoseStamped()
    goal_pose3.header.frame_id = 'map'
    goal_pose3.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose3.pose.position.x = -3.0
    goal_pose3.pose.position.y = 2.5
    goal_pose1.pose.orientation.x = 0.0
    goal_pose1.pose.orientation.y = 0.0
    goal_pose1.pose.orientation.z = 0.7068252
    goal_pose1.pose.orientation.w = 0.7073883

    navigator.waitUntilNav2Active()

    navigator.goToPose(goal_pose1)

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
    
    navigator.goToPose(goal_pose2)

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
    
    navigator.goToPose(goal_pose3)

    
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

    navigator.lifecycleShutdown()

    exit(0)


if __name__ == '__main__':
    main()