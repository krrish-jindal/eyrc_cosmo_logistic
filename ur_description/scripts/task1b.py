#!/usr/bin/env python3
import rclpy
import sys
import cv2
import math
import tf2_ros
import numpy as np
from rclpy.node import Node
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import TransformStamped
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import CompressedImage, Image
from cv2 import aruco
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Quaternion
from tf_transformations import quaternion_from_euler
import tf_transformations

class aruco_tf(Node):
	'''
	___CLASS___

	Description:    Class which servers purpose to define process for detecting aruco marker and publishing tf on pose estimated.
	'''

	def __init__(self):
		'''
		Description:    Initialization of class aruco_tf
						All classes have a function called __init__(), which is always executed when the class is being initiated.
						The __init__() function is called automatically every time the class is being used to create a new object.
						You can find more on this topic here -> https://www.w3schools.com/python/python_classes.asp
		'''

		super().__init__('aruco_tf_publisher')                                          # registering node
		self.cv_image = None  # Initialize cv_image as None
		self.depth_image = None
		############ Topic SUBSCRIPTIONS ############

		self.color_cam_sub = self.create_subscription(Image, '/camera/color/image_raw', self.colorimagecb, 10)
		self.depth_cam_sub = self.create_subscription(Image, '/camera/aligned_depth_to_color/image_raw', self.depthimagecb, 10)

		############ Constructor VARIABLES/OBJECTS ############

		image_processing_rate = 2.5                                                   # rate of time to process image (seconds)
		self.bridge = CvBridge()                                                        # initialise CvBridge object for image conversion
		self.tf_buffer = tf2_ros.buffer.Buffer()                                        # buffer time used for listening transforms
		self.listener = tf2_ros.TransformListener(self.tf_buffer, self)
		self.br = tf2_ros.TransformBroadcaster(self)   		                                 # object as transform broadcaster to send transform wrt some frame_id
		self.timer = self.create_timer(image_processing_rate, self.process_image)       # creating a timer based function which gets called on every 0.2 seconds (as defined by 'image_processing_rate' variable)
				
def main():
	'''
	Description:    Main function which creates a ROS node and spin around for the aruco_tf class to perform it's task
	'''

	rclpy.init(args=sys.argv)                                       # initialisation

	node = rclpy.create_node('aruco_tf_process')                    # creating ROS node

	node.get_logger().info('Node created: Aruco tf process')        # logging information

	aruco_tf_class = aruco_tf()                                     # creating a new object for class 'aruco_tf'

	rclpy.spin(aruco_tf_class)                                      # spining on the object to make it alive in ROS 2 DDS

	aruco_tf_class.destroy_node()                                   # destroy node after spin ends

	rclpy.shutdown()                                                # shutdown process


if __name__ == '__main__':
	'''
	Description:    If the python interpreter is running that module (the source file) as the main program, 
					it sets the special __name__ variable to have a value “__main__”. 
					If this file is being imported from another module, __name__ will be set to the module’s name.
					You can find more on this here -> https://www.geeksforgeeks.org/what-does-the-if-__name__-__main__-do/
	'''

	main()