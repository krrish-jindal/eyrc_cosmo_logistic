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
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
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

def detect_aruco(image):
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
		cam_mat = np.array([[931.1829833984375, 0.0, 640.0], [0.0, 931.1829833984375, 360.0], [0.0, 0.0, 1.0]],dtype=np.float32)

		# The distortion matrix is currently set to 0. 
		# We will be using it during Stage 2 hardware as Intel Realsense Camera provides these camera info.
		dist_mat = np.array([0.0,0.0,0.0,0.0,0.0])

		# We are using 150x150 aruco marker size
		size_of_aruco_cm = 15
		dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)

		arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
		arucoParams = cv2.aruco.DetectorParameters_create()
		(corners, aruco_id, rejected) = cv2.aruco.detectMarkers(image, arucoDict, parameters=arucoParams)
		for i in range(len(aruco_id)):

			x1= corners[i][0][0][0]
			y1= corners[i][0][0][1]
			x2= corners[i][0][2][0]
			y2= corners[i][0][2][1]
			#common point
			x3= corners[i][0][3][0]
			y3= corners[i][0][3][1]  

			coordinates= [x1,y1,x2,y2,x3,y3]
			area, width = calculate_rectangle_area(coordinates)
			rvecs, tvecs, trash =my_estimatePoseSingleMarkers(corners, size_of_aruco_cm, cam_mat, dist_mat)
			if area >= aruco_area_threshold:
				print("################################################")
				print(rvecs)
				rvec , tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners, size_of_aruco_cm, cam_mat, dist_mat)
				ids.append(int(aruco_id[i]))
				current_rvec = rvec[i]
				current_tvec = tvec[i]

				# center_aruco = current_tvec.squeeze()
				center_aruco = np.mean(corners[i][0], axis=0)
				center_x, center_y = map(int, center_aruco)
				
				distance_from_rgb = float(tvecs[i][2])

				center_aruco_list.append((center_x,center_y))
				distance_from_rgb_list.append(distance_from_rgb)


				#  Angle cal
				list_1 = current_rvec.tolist()
				# print(math.degrees(angle_aruco_list[i][2]))
				# print((angle_aruco_list[i][2]))
				rotation_mat, _ = cv2.Rodrigues(current_rvec)
				rotation_matrix, _ = cv2.Rodrigues(current_rvec)

				new = rotation_matrix[0].flatten().tolist()


				list=rotationMatrixToEulerAngles (rotation_mat)
				angle_aruco_list.append(list)
				flat_list.append(new)
				print(rotation_matrix)
				print("LLLLLLLLLLLLL")
				print(new)

				# angle_aruco_list = [item[0] for item in flat_list]


				

				width_aruco_list.append(width)
				cv2.aruco.drawDetectedMarkers(image, corners)
				# cv2.putText(image, f"Distance: {distance_from_rgb_list[i]:.2f} cm", (int(corners[i][0][0][0]), int(corners[i][0][0][1]) + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
				# cv2.putText(image, f"center{center_x, center_y}", (int(center_x), int(center_y)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
				cv2.putText(image, f"ID: {aruco_id[i]}", (int(corners[i][0][0][0]), int(corners[i][0][0][1]) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
				# cv2.putText(image, f"{angle_aruco_list[i]}", (int(center_x), int(center_y)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
				cv2.drawFrameAxes(image,cam_mat, dist_mat, current_rvec[0], current_tvec[0],4,4)	
				cv2.circle(image, (center_x ,center_y), 2, (255, 0, 0), 6)
		cv2.imshow("Aruco Detection", image)
		cv2.waitKey(1)  # Wait for 1ms
	
		############ ADD YOUR CODE HERE ############

		# INSTRUCTIONS & HELP : 

		#	->  Convert input BGR image to GRAYSCALE for aruco detection


		#   ->  Draw frame axes from coordinates received using pose estimation
		#       ->  HINT: You may use 'cv2.drawFrameAxes'

		############################################	
		# cv2.destroyAllWindows()
		return center_aruco_list, distance_from_rgb_list, angle_aruco_list, width_aruco_list, ids, tvec,rvec

def my_estimatePoseSingleMarkers(corners, marker_size, mtx, distortion):
	'''
	This will estimate the rvec and tvec for each of the marker corners detected by:
	   corners, ids, rejectedImgPoints = detector.detectMarkers(image)
	corners - is an array of detected corners for each detected marker in the image
	marker_size - is the size of the detected markers
	mtx - is the camera matrix
	distortion - is the camera distortion matrix
	RETURN list of rvecs, tvecs, and trash (so that it corresponds to the old estimatePoseSingleMarkers())
	'''
	marker_points = np.array([[-marker_size / 2, marker_size / 2, 0],
							  [marker_size / 2, marker_size / 2, 0],
							  [marker_size / 2, -marker_size / 2, 0],
							  [-marker_size / 2, -marker_size / 2, 0]], dtype=np.float32)
	trash = []
	rvecs = []
	tvecs = []
	i = 0
	for c in corners:
		nada, R, t = cv2.solvePnP(marker_points, corners[i], mtx, distortion, False, cv2.SOLVEPNP_IPPE_SQUARE)
		rvecs.append(R)
		tvecs.append(t)
		trash.append(nada)
	return rvecs, tvecs, trash

# def yawpitchrolldecomposition(R):

# 	sin_x    = math.sqrt(R[2,0] * R[2,0] +  R[2,1] * R[2,1])    
# 	validity  = sin_x < 1e-6
# 	if not singular:
# 		z1    = math.atan2(R[2,0], R[2,1])     # around z1-axis
# 		x      = math.atan2(sin_x,  R[2,2])     # around x-axis
# 		z2    = math.atan2(R[0,2], -R[1,2])    # around z2-axis
# 	else: # gimbal lock
# 		z1    = 0                                         # around z1-axis
# 		x      = math.atan2(sin_x,  R[2,2])     # around x-axis
# 		z2    = 0                                         # around z2-axis
	
# 	return np.array([[z1], [x], [z2]])

# yawpitchroll_angles = -180*yawpitchrolldecomposition(rmat)/math.pi
# yawpitchroll_angles[0,0] = (360-yawpitchroll_angles[0,0])%360 # change rotation sense if needed, comment this line otherwise
# yawpitchroll_angles[1,0] = yawpitchroll_angles[1,0]+90



def rotationMatrixToEulerAngles (R): 
	sy = math.sqrt(R[0, 0]* R[0, 0] + R[1, 0] * R[1, 0])
	singular =sy < 1e-6
	if not singular:
		x = math.atan2(R[2, 1], R[2,2])
		y = math.atan2(-R[2, 0], sy)
		z = math.atan2(R[1, 0], R[0, 0])
	else:
		x = math.atan2(-R[1, 2], R[1, 1])
		y = math.atan2(-R[2, 0], sy)
		z = 0
	return np.array([x, y, z])

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

		############ Topic SUBSCRIPTIONS ############

		self.color_cam_sub = self.create_subscription(Image, '/camera/color/image_raw', self.colorimagecb, 10)
		self.depth_cam_sub = self.create_subscription(Image, '/camera/aligned_depth_to_color/image_raw', self.depthimagecb, 10)

		############ Constructor VARIABLES/OBJECTS ############

		image_processing_rate = 2                                                  # rate of time to process image (seconds)
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

		############ ADD YOUR CODE HERE ############

		# INSTRUCTIONS & HELP : 

		#	->  Use data variable to convert ROS Image message to CV2 Image type

		#   ->  HINT: You may use CvBridge to do the same

		############################################


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
		# self.process_image()

		# result= detect_aruco(self.image)
		############ ADD YOUR CODE HERE ############

		# INSTRUCTIONS & HELP : 

		#	->  Use data variable to convert ROS Image message to CV2 Image type

		#   ->  HINT:   You may use CvBridge to do the same
		#               Check if you need any rotation or flipping image as input data maybe different than what you expect to be.
		#               You may use cv2 functions such as 'flip' and 'rotate' to do the same

		############################################


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
		focalX = 931.1829833984375
		focalY = 931.1829833984375
		
		center_list, distance_list, angle_list, width_list, ids,tvec,rvec = detect_aruco(self.cv_image)
		# Add your image processing code here
		for i in range(len(ids)):
	# Get the ArUco ID, distance, angle, and width for the current marker

			aruco_id = ids[i]
			distance = distance_list[i]
			angle_aruco = angle_list[i]
			width_aruco = width_list[i]
			center=center_list[i]
			# Correct the aruco angle using the correction formula
			angle_aruco = ((0.788 * angle_aruco[2]) - ((angle_aruco[2] ** 2) / 3160)) 
			
			roll, pitch, yaw =  0 , 0 ,(angle_aruco)
			q_rot = quaternion_from_euler(roll,pitch,yaw)


			# r = R.from_euler('xyz', [roll, pitch, yaw], degrees=True)
			# quaternion = r.as_quat()
			# # print("raw---",q_rot)
			# rot_matrix = np.eye(4)
			# rot_matrix[0:3, 0:3] = cv2.Rodrigues(np.array(rvec[i][0]))[0]
			# quat = tf_transformations.quaternion_from_matrix(rot_matrix)
			# Calculate the transform matrix (rotation and translation)
			# transform_matrix = np.eye(4)
			# transform_matrix[:3, :3] = r.as_matrix()
			# transform_matrix[:3, 3] = [tvec[i][0][0], tvec[i][0][1], tvec[i][0][2]]
			# print(q)
# 

			# Calculate quaternions from roll, pitch, and yaw (where roll and pitch are 0)
			# You can use the scipy library for this purpose
			# roll, pitch, yaw = 0, 0, angle_aruco  #pitch=y(green),roll=x(red),yaw=z(blue) 

			qy = q_rot[1]
			qz = q_rot[2]
			qx = q_rot[0]
			qw = q_rot[3]

			# qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
			# qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
			# qw = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
			# qz = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
			y = tvec[i][0][0]
			z = tvec[i][0][1]
			x = tvec[i][0][2]/100
			print(x,y,z)
			y_1 = (distance/100 * (sizeCamX - center_list[i][0] - centerCamX) / focalX)+0.02
			z_1 = (distance/100 * (sizeCamY - center_list[i][1] - centerCamY) / focalY)
			x_1 = distance/100
			print("GGGGGGG")
			print(x_1,y_1,z_1)



			print("@@@@@@@@@@@@@@@@@@")
			transform_msg = TransformStamped()
			# transform_msg.header.stamp = 
			# transform_msg.header.stamp = rclpy.node.get_clock().now().to_msg()
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
				# print(t)
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


			# Calculate the transform matrix (rotation and translation)
			# transform_matrix = np.eye(4)
			# transform_matrix[:3, :3] = r.as_matrix()
			# transform_matrix[:3, 3] = [tvec[i][0][0], tvec[i][0][1], tvec[i][0][2]]

			# # Create a TransformStamped message to publish the transform
			# transform_msg = TransformStamped()
			# transform_msg.header.stamp = node.get_clock().now().to_msg()
			# transform_msg.header.frame_id = 'camera_link'
			# transform_msg.child_frame_id = f'cam_{aruco_id}'
			# transform_msg.transform.translation.x = tvec[i][0][0] / 1000.0  # Convert mm to m
			# transform_msg.transform.translation.y = tvec[i][0][1] / 1000.0  # Convert mm to m
			# transform_msg.transform.translation.z = tvec[i][0][2] / 1000.0  # Convert mm to m
			# transform_msg.transform.rotation.x = quaternion[0]
			# transform_msg.transform.rotation.y = quaternion[1]
			# transform_msg.transform.rotation.z = quaternion[2]
			# transform_msg.transform.rotation.w = quaternion[3]

			# # Publish the transform
			# node.br.sendTransform(transform_msg)

			# # Lookup the transform between base_link and obj frame
			# try:
			# 	transform = node.tf_buffer.lookup_transform('base_link', transform_msg.child_frame_id, rclpy.time.Time())

			# 	# Publish the transform between object frame and base_link
			# 	obj_transform_msg = TransformStamped()
			# 	obj_transform_msg.header.stamp = node.get_clock().now().to_msg()
			# 	obj_transform_msg.header.frame_id = 'base_link'
			# 	obj_transform_msg.child_frame_id = f'obj_{aruco_id}'
			# 	obj_transform_msg.transform = transform.transform
			# 	node.br.sendTransform(obj_transform_msg)

			# except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
			# 	node.get_logger().error(f"Transform lookup failed for {transform_msg.child_frame_id}")

# Sh		ow the image with detected markers and center points

			
		
		
		# else:
			# print("No image received. Skipping image processing.")
		# print(center_list)

			



		############ ADD YOUR CODE HERE ############

		# INSTRUCTIONS & HELP : 

		#	->  Get aruco center, distance from rgb, angle, width and ids list from 'detect_aruco_center' defined above

		#   ->  Loop over detected box ids received to calculate position and orientation transform to publish TF 

		#   ->  Use this equation to correct the input aruco angle received from cv2 aruco function 'estimatePoseSingleMarkers' here
		#       It's a correction formula- 
		#       angle_aruco = (0.788*angle_aruco) - ((angle_aruco**2)/3160)

		#   ->  Then calculate quaternions from roll pitch yaw (where, roll and pitch are 0 while yaw is corrected aruco_angle)

		#   ->  Use center_aruco_list to get realsense depth and log them down. (divide by 1000 to convert mm to m)

		#   ->  Use this formula to rectify x, y, z based on focal length, center value and size of image
		#       x = distance_from_rgb * (sizeCamX - cX - centerCamX) / focalX
		#       y = distance_from_rgb * (sizeCamY - cY - centerCamY) / focalY
		#       z = distance_from_rgb
		#       where, 
		#               cX, and cY from 'center_aruco_list'
		#               distance_from_rgb is depth of object calculated in previous step
		#               sizeCamX, sizeCamY, centerCamX, centerCamY, focalX and focalY are defined above

		#   ->  Now, mark the center points on image frame using cX and cY variables with help of 'cv2.cirle' function 

		#   ->  Here, till now you receive coordinates from camera_link to aruco marker center position. 
		#       So, publish this transform w.r.t. camera_link using Geometry Message - TransformStamped 
		#       so that we will collect it's position w.r.t base_link in next step.
		#       Use the following frame_id-
		#           frame_id = 'camera_link'
		#           child_frame_id = 'cam_<marker_id>'          Ex: cam_20, where 20 is aruco marker ID

		#   ->  Then finally lookup transform between base_link and obj frame to publish the TF
		#       You may use 'lookup_transform' function to pose of obj frame w.r.t base_link 

		#   ->  And now publish TF between object frame and base_link
		#       Use the following frame_id-
		#           frame_id = 'base_link'
		#           child_frame_id = 'obj_<marker_id>'          Ex: obj_20, where 20 is aruco marker ID

		#   ->  At last show cv2 image window having detected markers drawn and center points located using 'cv2.imshow' function.
		#       Refer MD book on portal for sample image -> https://portal.e-yantra.org/

		#   ->  NOTE:   The Z axis of TF should be pointing inside the box (Purpose of this will be known in task 1B)
		#               Also, auto eval script will be judging angular difference aswell. So, make sure that Z axis is inside the box (Refer sample images on Portal - MD book)

		############################################


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