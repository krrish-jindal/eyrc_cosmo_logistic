def calculate_rectangle_area(coordinates):
		global area ,width

		height = ((coordinates[0] - coordinates[4])**2 + (coordinates[1] - coordinates[5])**2)**0.5
		width = ((coordinates[2] - coordinates[4])**2 + (coordinates[3] - coordinates[5])**2)**0.5

		# Calculate the area
		area = width * height

		return area, width, height

def detect_aruco(image):


		center_aruco_list = []
		distance_from_rgb_list = []
		angle_aruco_list = []
		width_aruco_list = []
		ids = []

		aruco_area_threshold = 1500

		# The camera matrix is defined as per camera info loaded from the plugin used. 
		# You may get this from /camer_info topic when camera is spawned in gazebo.
		# Make sure you verify this matrix once if there are calibration issues.
		cam_mat = np.array([[931.1829833984375, 0.0, 640.0], [0.0, 931.1829833984375, 360.0], [0.0, 0.0, 1.0]])

		# The distortion matrix is currently set to 0. 
		# We will be using it during Stage 2 hardware as Intel Realsense Camera provides these camera info.
		dist_mat = np.array([0.0,0.0,0.0,0.0,0.0])

		# We are using 150x150 aruco marker size
		size_of_aruco_cm = 15
		dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)

		arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
		arucoParams = cv2.aruco.DetectorParameters_create()
		(corners, id, rejected) = cv2.aruco.detectMarkers(image, arucoDict, parameters=arucoParams)
		
		for i in range(len(id)):
			x1= corners[i][0][0][0]
			y1= corners[i][0][0][1]
			x2= corners[i][0][2][0]
			y2= corners[i][0][2][1]
			#common point
			x3= corners[i][0][3][0]
			y3= corners[i][0][3][1]  

			coordinates= [x1,y1,x2,y2,x3,y3]
			calculate_rectangle_area(coordinates)
			if area >= aruco_area_threshold:
				rvec , tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners, size_of_aruco_cm, cam_mat, dist_mat)
				ids.append(i)

				current_rvec = rvec[i]
				current_tvec = tvec[i]
				
				# center_aruco = current_tvec.squeeze()
				center_aruco = np.mean(corners[i][0], axis=0)
				center_x, center_y = map(int, center_aruco)
	
				distance_from_rgb = cv2.norm(current_tvec)
	
				# Calculate the angle of the ArUco marker based on the rotation vector (rvec)
				# You may need to convert the rotation vector to Euler angles or other representations depending on your needs
				# Here's an example assuming rvec contains Euler angles:
	
				# Append the values to their respective lists
				angle_aruco = current_rvec.squeeze()
	
				center_aruco_list.append(center_aruco)
				distance_from_rgb_list.append(distance_from_rgb)
				angle_aruco_list.append(angle_aruco)
				width_aruco_list.append(width)
				cv2.aruco.drawDetectedMarkers(image, corners)
				# cv2.putText(image, f"Distance: {distance_from_rgb_list[i]:.2f} cm", (int(corners[i][0][0][0]), int(corners[i][0][0][1]) + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
				cv2.putText(image, f"center{center_x, center_y}", (int(center_x), int(center_y)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
				cv2.putText(image, f"ID: {id[i]}", (int(corners[i][0][0][0]), int(corners[i][0][0][1]) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
				cv2.circle(image, (center_x ,center_y), 2, (255, 0, 0), 6)
		cv2.imshow("Aruco Detection", image)
		cv2.waitKey(1)  # Wait for 1ms

		print(width_aruco_list)
		return center_aruco_list, distance_from_rgb_list, angle_aruco_list, width_aruco_list, ids