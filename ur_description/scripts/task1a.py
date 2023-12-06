#!/usr/bin/env python3


'''
*****************************************************************************************
*
*        		===============================================
*           		    Cosmo Logistic (CL) Theme (eYRC 2023-24)
*        		===============================================
*
*  This script should be used to implement Task 1A of Cosmo Logistic (CL) Theme (eYRC 2023-24).
*
*  This software is made available on an "AS IS WHERE IS BASIS".
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or
*  breach of the terms of this agreement.
*
*****************************************************************************************
'''

# Team ID:          [ Team-ID ]
# Author List:		[ Names of team members worked on this file separated by Comma: Name1, Name2, ... ]
# Filename:		    task1a.py
# Functions:
#			        [ Comma separated list of functions in this file ]
# Nodes:		    Add your publishing and subscribing node
#                   Example:
#			        Publishing Topics  - [ /tf ]
#                   Subscribing Topics - [ /camera/aligned_depth_to_color/image_raw, /etc... ]


################### IMPORT MODULES #######################

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


##################### FUNCTION DEFINITIONS #######################
def calculate_rectangle_area(coordinates):
		# global width
		'''
		Description: Function to calculate the area of a detected ArUco marker

		Args:
			coordinates (list): coordinates of detected ArUco (4 sets of (x, y) coordinates)

		Returns:
			area (float): area of detected ArUco
			width (float): width of detected ArUco
			height (float): height of detected ArUco
		'''
		height = ((coordinates[0] - coordinates[4])**2 + (coordinates[1] - coordinates[5])**2)**0.5
		width = ((coordinates[2] - coordinates[4])**2 + (coordinates[3] - coordinates[5])**2)**0.5

		# Calculate the area
		area = width * height

		return area, width 

def detect_aruco(image, depth):
		'''
		Description:    Function to perform aruco detection and return each detail of aruco detected 
						such as marker ID, distance, angle, width, center point location, etc.

		Args:
			image                   (Image):    Input image frame received from respective camera topic

		Returns:
			center_aruco_list       (list):     Center points of all aruco markers detected
			distance_from_rgb_list  (list):     Distance value of each aruco markers detected from RGB camera
			angle_aruco_list        (list):     Angle of all pose estimated for aruco marker
			width_aruco_list        (list):     Width of all detected aruco markers
			ids                     (list):     List of all aruco marker IDs detected in a single frame 
		'''

		center_aruco_list = []
		distance_from_rgb_list = []
		angle_aruco_list = []
		width_aruco_list = []
		ids = []
		flat_list =[]
		aruco_area_threshold = 1500


		# The camera matrix is defined as per camera info loaded from the plugin used. 
		# You may get this from /camer_info topic when camera is spawned in gazebo.
		# Make sure you verify this matrix once if there are calibration issues.
		cam_mat = np.array([[915.3003540039062, 0.0, 642.724365234375], [0.0, 914.0320434570312, 361.9780578613281], [0.0, 0.0, 1.0]],dtype=np.float32)

		# The distortion matrix is currently set to 0. 
		# We will be using it during Stage 2 hardware as Intel Realsense Camera provides these camera info.
		dist_mat = np.array([0.0,0.0,0.0,0.0,0.0])

		# We are using 150x150 aruco marker size
		size_of_aruco_cm = 15
		dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
		arucoParams = cv2.aruco.DetectorParameters()
		brightness = 1.0 
		contrast = 2.0 
		image = cv2.medianBlur(image, 3)
		image = cv2.addWeighted(image, contrast, np.zeros(image.shape, image.dtype), 0, brightness)
		image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
		try:
			(corners, aruco_id, rejected) = cv2.aruco.detectMarkers(image, dictionary, parameters=arucoParams)
			for i in range(len(aruco_id)):

				x1= corners[i][0][0][0]
				y1= corners[i][0][0][1]
				x2= corners[i][0][2][0]
				y2= corners[i][0][2][1]
				x3= corners[i][0][3][0]
				y3= corners[i][0][3][1]  

				coordinates= [x1,y1,x2,y2,x3,y3]
				area, width = calculate_rectangle_area(coordinates)

				if area >= aruco_area_threshold:
					rvec , tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners, size_of_aruco_cm, cam_mat, dist_mat)
					ids.append(int(aruco_id[i]))
					current_rvec = rvec[i]
					current_tvec = tvec[i]
					
					# center_aruco = current_tvec.squeeze()
					center_aruco = np.mean(corners[i][0], axis=0)
					center_x, center_y = map(int, center_aruco)
					distance_from_rgb = cv2.norm(current_tvec)
					center_aruco_list.append((center_x,center_y))
					depth_data = depth[center_y, center_x]
					distance_from_rgb_list.append(depth_data)

					#  Angle cal
					list_1 = current_rvec.tolist()
					flat_list.append(list_1)
					angle_aruco_list = [item[0] for item in flat_list]
					rotation_mat, _ = cv2.Rodrigues(current_rvec)
					
					width_aruco_list.append(width)
					cv2.aruco.drawDetectedMarkers(image, corners)
					# cv2.putText(image, f"Distance: {distance_from_rgb_list[i]:.2f} cm", (int(corners[i][0][0][0]), int(corners[i][0][0][1]) + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
					# cv2.putText(image, f"center{center_x, center_y}", (int(center_x), int(center_y)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
					cv2.putText(image, f"ID: {aruco_id[i]}", (int(corners[i][0][0][0]), int(corners[i][0][0][1]) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
					# cv2.putText(image, f"{angle_aruco_list[i]}", (int(center_x), int(center_y)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
					cv2.drawFrameAxes(image,cam_mat, dist_mat, current_rvec[0] , current_tvec[0],2,1)	
					cv2.circle(image, (center_x ,center_y), 2, (255, 0, 0), 6)
			cv2.imshow("Aruco Detection", image)
			cv2.waitKey(1)  # Wait for 1ms
			# cv2.destroyAllWindows()
			return center_aruco_list, distance_from_rgb_list, angle_aruco_list, width_aruco_list, ids, tvec
		except:
			pass

##################### CLASS DEFINITION #######################

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

	def depthimagecb(self, data):
		'''
		Description:    Callback function for aligned depth camera topic. 
						Use this function to receive image depth data and convert to CV2 image

		Args:
			data (Image):    Input depth image frame received from aligned depth camera topic

		Returns:
		'''
		br = CvBridge()

		self.depth_image = br.imgmsg_to_cv2(data, data.encoding)

	def colorimagecb(self, data):

		'''
		Description:    Callback function for colour camera raw topic.
						Use this function to receive raw image data and convert to CV2 image

		Args:
			data (Image):    Input coloured raw image frame received from image_raw camera topic

		Returns:
		'''
		br = CvBridge()

		self.cv_image = br.imgmsg_to_cv2(data, "bgr8")
		self.gray = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2GRAY)
	
	def process_image(self):
		'''
		Description:    Timer function used to detect aruco markers and publish tf on estimated poses.

		Args:
		Returns:
		'''

		############ Function VARIABLES ############

		# These are the variables defined from camera info topic such as image pixel size, focalX, focalY, etc.
		# Make sure you verify these variable values once. As it may affect your result.
		# You can find more on these variables here -> http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/CameraInfo.html
		
		sizeCamX = 1280
		sizeCamY = 720
		centerCamX = 640 
		centerCamY = 360
		focalX = 915.30035
		focalY = 915.300354
		
		center_list, distance_list, angle_list, width_list, ids,tvec = detect_aruco(self.cv_image, self.depth_image)
		rpy = []
		list_rpy = []

		#  image processing code 
		for i in range(len(ids)):
			aruco_id = ids[i]
			distance = distance_list[i]
			angle_aruco = angle_list[i]
			width_aruco = width_list[i]
			center=center_list[i]
			print(angle_aruco)
			angle_aruco = ((0.788*angle_aruco[2]) - ((angle_aruco[2]**2)/3160))
			if round(angle_aruco) == 0:
				angle_aruco = (angle_aruco) + math.pi
				roll, pitch, yaw = 0.4, 0, angle_aruco
			else:
				roll, pitch, yaw = 0, -0.261, angle_aruco

			print(angle_aruco)

			rpy.append(roll)
			rpy.append(pitch)
			rpy.append(yaw)
			list_rpy.append(rpy)
			rpy_np = np.array(list_rpy)
			rot_mat, _ = cv2.Rodrigues(rpy_np)

			transform_matrix = np.array([[0, 0, 1],
								  [-1, 0, 0],
								  [0, 1, 0]])
			
			good_mat = np.dot(rot_mat, transform_matrix)

			euler_good = tf_transformations.euler_from_matrix(good_mat)

			euler_list = euler_good
			roll = euler_list[0]
			pitch = euler_list[1]
			yaw = euler_list[2]
			list_rpy = []
			rpy = []

			q_rot = quaternion_from_euler(roll,pitch,yaw)
			qy = q_rot[1]
			qz = q_rot[2]
			qx = q_rot[0]
			qw = q_rot[3]

			x = tvec[i][0][0]
			y = tvec[i][0][1]
			z = tvec[i][0][2]
			y_1 = distance/1000 * (sizeCamX - center_list[i][0] - centerCamX) / focalX
			z_1 = distance/1000 * (sizeCamY - center_list[i][1] - centerCamY) / focalY
			x_1 = distance/1000

			transform_msg = TransformStamped()
			transform_msg.header.stamp = self.get_clock().now().to_msg()
			transform_msg.header.frame_id = 'camera_link'
			transform_msg.child_frame_id = f'cam_{aruco_id}'
			transform_msg.transform.translation.x = x_1
			transform_msg.transform.translation.y = y_1
			transform_msg.transform.translation.z = z_1
			transform_msg.transform.rotation.x = qx
			transform_msg.transform.rotation.y = qy
			transform_msg.transform.rotation.z = qz
			transform_msg.transform.rotation.w = qw
			self.br.sendTransform(transform_msg)

			try:
				t = self.tf_buffer.lookup_transform('base_link', transform_msg.child_frame_id, rclpy.time.Time())
				transform_msg.header.stamp = self.get_clock().now().to_msg()

				transform_msg.header.frame_id = 'base_link'
				transform_msg.child_frame_id = f'obj_{aruco_id}'
				transform_msg.transform.translation.x = t.transform.translation.x
				transform_msg.transform.translation.y = t.transform.translation.y
				transform_msg.transform.translation.z = t.transform.translation.z
				transform_msg.transform.rotation.x = t.transform.rotation.x
				transform_msg.transform.rotation.y = t.transform.rotation.y
				transform_msg.transform.rotation.z = t.transform.rotation.z
				transform_msg.transform.rotation.w = t.transform.rotation.w
				self.br.sendTransform(transform_msg)
			except:
				pass

##################### FUNCTION DEFINITION #######################

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