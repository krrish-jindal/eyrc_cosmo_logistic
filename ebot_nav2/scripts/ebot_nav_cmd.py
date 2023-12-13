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
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import Odometry
import time

class NavigationController(Node):

    def __init__(self):
        rclpy.init()  # Initialize rclpy here
        super().__init__('nav_dock')

        self.attach = self.create_client(srv_type=AttachLink, srv_name='/ATTACH_LINK')
        self.detach = self.create_client(srv_type=DetachLink, srv_name='/DETACH_LINK')
        self.client_docking = self.create_client(srv_type=DockSw, srv_name='dock_control')
        self.vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odometry_callback, 10)
        self.vel_msg = Twist()
        self.navigator = BasicNavigator()

        self.robot_pose = [0, 0]

    def odometry_callback(self, msg):
        self.robot_pose[0] = msg.pose.pose.position.x
        self.robot_pose[1] = msg.pose.pose.position.y
        print(self.robot_pose)

    def send_request(self, orientation):
        request_dock = DockSw.Request()
        request_dock.orientation_dock = True
        request_dock.orientation = orientation
        future = self.client_docking.call_async(request_dock)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def rack_attach(self, rack):
        req = AttachLink.Request()
        req.model1_name = 'ebot'
        req.link1_name = 'ebot_base_link'
        req.model2_name = rack
        req.link2_name = 'link'

        atc = self.attach.call_async(req)
        rclpy.spin_until_future_complete(self, atc)
        self.vel_msg.linear.x = 0.2
        self.vel_pub.publish(self.vel_msg)
        time.sleep(2)
        self.vel_msg.linear.x = 0.0
        self.vel_pub.publish(self.vel_msg)
        return atc.result()

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

    def rack_detach(self, rack):
        req = DetachLink.Request()
        req.model1_name = 'ebot'
        req.link1_name = 'ebot_base_link'
        req.model2_name = rack
        req.link2_name = 'link'

        dtc = self.detach.call_async(req)
        rclpy.spin_until_future_complete(self, dtc)
        self.vel_msg.linear.x = 0.2
        self.vel_pub.publish(self.vel_msg)
        time.sleep(2)
        self.vel_msg.linear.x = 0.0
        self.vel_pub.publish(self.vel_msg)
        return dtc.result()

    def navigate_and_dock(self, goal_pick, goal_drop, orientation_rack, rack):
        self.navigator.goToPose(goal_pick)
        self.nav_reach(goal_pick)
        while abs(goal_pick.pose.position.x - self.robot_pose[0]) > 0.01:
            error = goal_pick.pose.position.x - self.robot_pose[0]
            self.vel_msg.linear.x = error * 2
            self.vel_pub.publish(self.vel_msg)

        self.vel_msg.linear.x = 0.0
        self.vel_pub.publish(self.vel_msg)

        print("POSE----",self.robot_pose)
        self.send_request(orientation_rack)
        self.rack_attach(rack)

        self.navigator.goToPose(goal_drop)
        self.nav_reach(goal_drop)
        self.rack_detach(rack)

    def main(self):
        package_name = 'ebot_nav2'
        config = "config/config.yaml"

        ebot_nav2_dir = get_package_share_directory('ebot_nav2')

        pkg_share = FindPackageShare(package=package_name).find(package_name)
        config_path = os.path.join(pkg_share, config)
        with open(config_path, 'r') as infp:
            pos_rack = infp.read()

        data_dict = yaml.safe_load(pos_rack)

        positions = data_dict['position']
        rack1_coordinates = positions[0]['rack1']
        rack2_coordinates = positions[1]['rack2']
        rack3_coordinates = positions[2]['rack3']
        package_id = data_dict['package_id'][0]

        orientation_rack_1 = rack1_coordinates[2]
        orientation_rack_2 = rack2_coordinates[2]
        orientation_rack_3 = rack3_coordinates[2]
        rack_list = ["rack1", "rack2", "rack3"]

        goal_pick_1 = PoseStamped()
        goal_pick_1.header.frame_id = 'map'
        goal_pick_1.header.stamp = self.navigator.get_clock().now().to_msg()
        goal_pick_1.pose.position.x = 0.108200
        goal_pick_1.pose.position.y = rack1_coordinates[1]
        goal_pick_1.pose.orientation.x = 0.0
        goal_pick_1.pose.orientation.y = 0.0
        goal_pick_1.pose.orientation.z = 0.7077099
        goal_pick_1.pose.orientation.w = 0.7065031

        # Define other goals...

        goal_drop_1 = PoseStamped()
        goal_drop_1.header.frame_id = 'map'
        goal_drop_1.header.stamp = self.navigator.get_clock().now().to_msg()
        goal_drop_1.pose.position.x = 0.5
        goal_drop_1.pose.position.y = -2.455
        goal_drop_1.pose.orientation.x = 0.0
        goal_drop_1.pose.orientation.y = 0.0
        goal_drop_1.pose.orientation.z = 0.9999997
        goal_drop_1.pose.orientation.w = 0.0007963



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
        goal_pick_3.header.stamp = self.navigator.get_clock().now().to_msg()
        goal_pick_3.pose.position.x = 1.998759
        goal_pick_3.pose.position.y = -7.102119
        goal_pick_3.pose.orientation.x = 0.0
        goal_pick_3.pose.orientation.y = 0.0
        goal_pick_3.pose.orientation.z = 0.0
        goal_pick_3.pose.orientation.w = 1.0



        goal_drop_2 = PoseStamped()
        goal_drop_2.header.frame_id = 'map'
        goal_drop_2.header.stamp = self.navigator.get_clock().now().to_msg()
        goal_drop_2.pose.position.x = 1.650000
        goal_drop_2.pose.position.y = -3.684832
        goal_drop_2.pose.orientation.x = 0.0
        goal_drop_2.pose.orientation.y = 0.0
        goal_drop_2.pose.orientation.z =  -0.70398
        goal_drop_2.pose.orientation.w =  0.7102198

        goal_drop_3 = PoseStamped()
        goal_drop_3.header.frame_id = 'map'
        goal_drop_3.header.stamp = self.navigator.get_clock().now().to_msg()
        goal_drop_3.pose.position.x =1.550000
        goal_drop_3.pose.position.y = -1.298586
        goal_drop_3.pose.orientation.x = 0.0
        goal_drop_3.pose.orientation.y = 0.0
        goal_drop_3.pose.orientation.z =  0.6921004
        goal_drop_3.pose.orientation.w = 0.7218012 


        # Define other drop goals...

        self.navigator.waitUntilNav2Active()

        if package_id == 3:
            self.navigate_and_dock(goal_pick_3, goal_drop_3, orientation_rack_3, rack_list[2])
        elif package_id == 2:
            # Navigate for package_id 2
            pass
        else:
            # Navigate for package_id 1
            pass

        exit(0)

if __name__ == '__main__':
    nav_controller = NavigationController()
    nav_controller.main()
