#!/usr/bin/env python3

import rospy
import numpy as np
import cv2
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Pose
from cv_bridge import CvBridge, CvBridgeError
from tf.transformations import quaternion_from_euler
from math import pi
from detection_msgs.msg import Detection2DArray, Detection2D

class DepthCamera:
	def __init__(self):
		rospy.init_node('Plate_Detector')

		# Initialize CvBridge
		self.bridge = CvBridge()

		# Subscriber for the color image to isolate red
		self.color_sub_red = rospy.Subscriber('/camera/color/image_raw', Image, self.color_callback)

		# Subscriber for the depth image
		self.depth_sub = rospy.Subscriber('/camera/aligned_depth_to_color/image_raw', Image, self.depth_callback)

		# Publisher for output image
		self.image_pub = rospy.Publisher('/output_image', Image, queue_size=1)

		# Publisher for yellow mask
		self.yellow_mask_pub = rospy.Publisher('/mask/yellow', Image, queue_size=1)

		# Publisher for green mask
		self.green_mask_pub = rospy.Publisher('/mask/green', Image, queue_size=1)

		# Publisher for object pose
		self.yellow_object_pub = rospy.Publisher('/pose_list/yellow', Detection2DArray, queue_size=1)

		# Publisher for object pose
		self.green_object_pub = rospy.Publisher('/pose_list/green', Detection2DArray, queue_size=1)

		# Subscriber for depth camera's intrinsic parameters
		camera_info = rospy.wait_for_message('/camera/aligned_depth_to_color/camera_info', CameraInfo)
		self.fx = camera_info.K[0]
		self.fy = camera_info.K[4]
		self.pp_cx = camera_info.K[2]
		self.pp_cy = camera_info.K[5]

	def color_callback(self, color_data):
		try:
			color_image = self.bridge.imgmsg_to_cv2(color_data, "bgr8")
		except CvBridgeError as e:
			print(e)

		# color_image_copy = color_image.copy()

		color_image_HSV = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)

		#Isolate yellow objects
		lower_yellow = np.array([0, 80, 130], dtype="uint8")
		upper_yellow = np.array([255, 220, 255], dtype="uint8")
		mask = cv2.inRange(color_image_HSV, lower_yellow, upper_yellow)
		output = cv2.bitwise_and(color_image_HSV, color_image_HSV, mask=mask)

		kernel = np.ones((3,3),np.uint8)
		erosion = cv2.erode(output, kernel, iterations = 3)
		dilate = cv2.dilate(erosion, kernel, iterations = 2)

		yellow_gray = cv2.cvtColor(dilate, cv2.COLOR_BGR2GRAY)

		self.yellow_mask_pub.publish(self.bridge.cv2_to_imgmsg(dilate, encoding="bgr8"))

		self.yellow_gray = yellow_gray

		contours, hierarchy = cv2.findContours(yellow_gray, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
		contours = [cnt for cnt in contours if cv2.contourArea(cnt) > 2000]

		for cnt in contours:
			cv2.drawContours(color_image, [cnt], -1, (255,0,0), 3)
			M = cv2.moments(cnt)
			cx = int(M['m10']/M['m00'])
			cy = int(M['m01']/M['m00'])
			cv2.circle(color_image,(cx,cy), 5, (255,0,0), -1)

		#Isolate green objects
		lower_green = np.array([40, 10, 80], dtype="uint8")
		upper_green = np.array([95, 200, 200], dtype="uint8")
		mask = cv2.inRange(color_image_HSV, lower_green, upper_green)
		output = cv2.bitwise_and(color_image_HSV, color_image_HSV, mask=mask)

		kernel = np.ones((3,3),np.uint8)
		erosion = cv2.erode(output, kernel, iterations = 1)
		# dilate = cv2.dilate(erosion, kernel)

		green_gray = cv2.cvtColor(erosion, cv2.COLOR_BGR2GRAY)

		self.green_mask_pub.publish(self.bridge.cv2_to_imgmsg(erosion, encoding="bgr8"))

		self.green_gray = green_gray

		contours, hierarchy = cv2.findContours(green_gray, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
		contours = [cnt for cnt in contours if cv2.contourArea(cnt) > 2000]

		for cnt in contours:
			cv2.drawContours(color_image, [cnt], -1, (0,255,0), 3)
			M = cv2.moments(cnt)
			cx = int(M['m10']/M['m00'])
			cy = int(M['m01']/M['m00'])
			cv2.circle(color_image,(cx,cy), 5, (0,255,0), -1)

		self.image_pub.publish(self.bridge.cv2_to_imgmsg(color_image, "bgr8"))

	def ExtractPose(self, gray_data, depth_image):
		contours, hierarchy = cv2.findContours(gray_data, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
		contours = [cnt for cnt in contours if cv2.contourArea(cnt) > 2000]

		object_array = Detection2DArray()
		object_array.header.stamp = rospy.Time.now()
		object_array.header.frame_id = "camera_color_optical_frame"

		tray_list = []

		for cnt in contours:
			M = cv2.moments(cnt)
			cx = int(M['m10']/M['m00'])
			cy = int(M['m01']/M['m00'])

			object_pose = Detection2D()
			object_pose.header.stamp = rospy.Time.now()
			object_pose.header.frame_id = "camera_color_optical_frame"

			#Bounding box not present for this code but is needed for Detection2D
			object_pose.bbox.center.x = 0				
			object_pose.bbox.center.y = 0
			object_pose.bbox.center.theta = 0
			object_pose.results.score = 1

			if cv2.contourArea(cnt) < 3000:
				continue #ignoring small contours
			elif cv2.contourArea(cnt)  < 5000:				
				object_pose.results.Class = "cup"
			elif cv2.contourArea(cnt)  < 10000:
				object_pose.results.Class = "bowl"
			else:
				object_pose.results.Class = "plate"

			contour_pose = Pose()
			
			depth = depth_image[cy, cx]/1000

			x_meters = (cx - self.pp_cx) * depth / self.fx
			y_meters = (cy - self.pp_cy) * depth / self.fy
			z_meters = depth

			contour_pose.position.x = x_meters
			contour_pose.position.y = y_meters
			contour_pose.position.z = z_meters

			contour_pose_quaternion = quaternion_from_euler(0, 0, 0)
			contour_pose.orientation.x = contour_pose_quaternion[0]				
			contour_pose.orientation.y = contour_pose_quaternion[1]
			contour_pose.orientation.z = contour_pose_quaternion[2]
			contour_pose.orientation.w = contour_pose_quaternion[3]

			object_pose.results.pose = contour_pose

			object_array.detections.append(object_pose)

		return object_array

	def depth_callback(self, depth_data):
		try:
			depth_image = self.bridge.imgmsg_to_cv2(depth_data, "32FC1")
		except CvBridgeError as e:
			print(e)

		if self.yellow_gray.any():

			yellow_object_array = self.ExtractPose(self.yellow_gray, depth_image)
			self.yellow_object_pub.publish(yellow_object_array)

		if self.green_gray.any():

			green_object_array = self.ExtractPose(self.green_gray, depth_image)
			self.green_object_pub.publish(green_object_array)


if __name__ == '__main__':
	try:
		depth_camera = DepthCamera()
		rospy.spin()
	except KeyboardInterrupt:
		print('Shutting down the ROS Plate Detector Node')
		cv2.destroyAllWindows()