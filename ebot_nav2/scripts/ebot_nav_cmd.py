#! /usr/bin/env python3
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from ebot_docking.srv import DockSw
from linkattacher_msgs.srv import AttachLink
from linkattacher_msgs.srv import DetachLink
from rclpy.qos import QoSProfile
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time



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

def rack_detach(rack):

    req = DetachLink.Request()
    req.model1_name =  'ebot'      
    req.link1_name  = 'ebot_base_link'       
    req.model2_name =  rack       
    req.link2_name  = 'link'
    print("Detaching")
    dtc=detach.call_async(req)
    rclpy.spin_until_future_complete(node, dtc)
    vel_msg.linear.x=0.8
    vel_pub.publish(vel_msg)
    print(vel_msg)
    time.sleep(3)
    vel_msg.linear.x=0.0
    vel_pub.publish(vel_msg)
    return dtc.result()

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


def move_with_linear_x(duration, linear_x, angular_z):
    start_time = time.time()  # Get the current time
    end_time = start_time + duration

    # Create a Twist message to control the linear.x (forward movement)
    vel_msg = Twist()
    vel_msg.linear.x = linear_x
    vel_msg.angular.z = angular_z

    while time.time() < end_time:
        vel_pub.publish(vel_msg)  # Publish the Twist message to control the robot's movement

    # Stop the forward movement by setting linear.x to 0
    vel_msg.linear.x = 0.0
    vel_msg.angular.z = 0.0
    vel_pub.publish(vel_msg)



def publish_vel_callback():
    # Publish the Twist message to control the robot's movement
    vel_pub.publish(vel_msg)

def main():
    global client_docking, node ,attach ,detach, navigator ,vel_msg, vel_pub
    rclpy.init()

    node = rclpy.create_node("nav_dock")
    attach = node.create_client(srv_type=AttachLink, srv_name='/ATTACH_LINK')
    detach = node.create_client(srv_type=DetachLink, srv_name='/DETACH_LINK')
    client_docking = node.create_client(srv_type=DockSw, srv_name='dock_control')
    vel_pub = node.create_publisher(Twist, "/cmd_vel", 10)
    vel_msg = Twist()
    navigator = BasicNavigator()

    publish_rate = 10  # Adjust this value as needed (e.g., 10 Hz)
    timer_period = 1.0 / publish_rate  # Calculate the timer period in seconds
    node.create_timer(timer_period, publish_vel_callback)

    
    orientation_rack_1 = 3.139999
    orientation_rack_2 = -1.570000
    orientation_rack_3 = 1.569999
    rack_list=["rack1","rack2","rack3"]

    goal_pick_1 = PoseStamped()
    goal_pick_1.header.frame_id = 'map'
    goal_pick_1.header.stamp = navigator.get_clock().now().to_msg()
    goal_pick_1.pose.position.x = -0.038125
    goal_pick_1.pose.position.y = 4.351180
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
    goal_drop_1.pose.position.x = 0.798571
    goal_drop_1.pose.position.y = -2.330280
    goal_drop_1.pose.orientation.x = 0.0
    goal_drop_1.pose.orientation.y = 0.0
    goal_drop_1.pose.orientation.z = -0.9999787
    goal_drop_1.pose.orientation.w =  0.0065288 

    goal_drop_br_1 = PoseStamped()
    goal_drop_br_1.header.frame_id = 'map'
    goal_drop_br_1.header.stamp = navigator.get_clock().now().to_msg()
    goal_drop_br_1.pose.position.x =-0.198191
    goal_drop_br_1.pose.position.y = -2.330280
    goal_drop_br_1.pose.orientation.x = 0.0
    goal_drop_br_1.pose.orientation.y = 0.0
    goal_drop_br_1.pose.orientation.z =  -0.9999787
    goal_drop_br_1.pose.orientation.w = 0.0065288 
    navigator.waitUntilNav2Active()
    
    goal_drop_2 = PoseStamped()
    goal_drop_2.header.frame_id = 'map'
    goal_drop_2.header.stamp = navigator.get_clock().now().to_msg()
    goal_drop_2.pose.position.x = 1.695165
    goal_drop_2.pose.position.y = -3.825960
    goal_drop_2.pose.orientation.x = 0.0
    goal_drop_2.pose.orientation.y = 0.0
    goal_drop_2.pose.orientation.z =  -0.7118631
    goal_drop_2.pose.orientation.w =  0.7023182
  
    goal_drop_br_2 = PoseStamped()
    goal_drop_br_2.header.frame_id = 'map'
    goal_drop_br_2.header.stamp = navigator.get_clock().now().to_msg()
    goal_drop_br_2.pose.position.x =1.401671
    goal_drop_br_2.pose.position.y = -3.998043
    goal_drop_br_2.pose.orientation.x = 0.0
    goal_drop_br_2.pose.orientation.y = 0.0
    goal_drop_br_2.pose.orientation.z =  -0.7118631
    goal_drop_br_2.pose.orientation.w =  0.7023182
    navigator.waitUntilNav2Active()
   
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

    goal_drop_br_3 = PoseStamped()
    goal_drop_br_3.header.frame_id = 'map'
    goal_drop_br_3.header.stamp = navigator.get_clock().now().to_msg()
    goal_drop_br_3.pose.position.x =1.369186
    goal_drop_br_3.pose.position.y = -0.861295
    goal_drop_br_3.pose.orientation.x = 0.0
    goal_drop_br_3.pose.orientation.y = 0.0
    goal_drop_br_3.pose.orientation.z =  0.6921004
    goal_drop_br_3.pose.orientation.w = 0.7218012 
    navigator.waitUntilNav2Active()

# Pick Rack_1
    navigator.goToPose(goal_pick_1)
    nav_reach(1)
    send_request(orientation_rack_1)
    rack_attach(rack_list[0])

# Drop Rack_1
    navigator.goToPose(goal_drop_br_1)
    nav_reach(2)
    navigator.goToPose(goal_drop_1)
    nav_reach(2)
    rack_detach(rack_list[0])
    move_with_linear_x(3,0.5,-1.0)


# Pick Rack_2
    navigator.goToPose(goal_pick_2)
    nav_reach(3)
    move_with_linear_x(1,0.1,0.0)
    send_request(orientation_rack_2)
    rack_attach(rack_list[1])
    move_with_linear_x(2,0.1,-0.2)

#  Drop Rack_2
    navigator.goToPose(goal_drop_br_2)
    nav_reach(4)
    navigator.goToPose(goal_drop_2)
    nav_reach(4)
    rack_detach(rack_list[1])
    move_with_linear_x(2,0.1,0.0)


# Pick Rack_3
    navigator.goToPose(goal_pick_3)
    nav_reach(5)
    send_request(orientation_rack_3)
    rack_attach(rack_list[2])
    move_with_linear_x(1,0.1,-0.2)
    
#  Drop Rack_3
    navigator.goToPose(goal_drop_br_3)
    nav_reach(6)
    navigator.goToPose(goal_drop_3)
    nav_reach(6)
    rack_detach(rack_list[2])
    move_with_linear_x(2,0.1,0.0)
    

    navigator.lifecycleShutdown()

    exit(0)


if __name__ == '__main__':
    main()