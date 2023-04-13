#!/usr/bin/env python3

import rospy
import numpy as np
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class DepthCamera:
	def __init__(self):
		rospy.init_node('ContourFinder')

		# Initialize CvBridge
		self.bridge = CvBridge()

		# Subscriber for the color image to isolate red
		self.color_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.color_callback)

		# Publisher for canny contour
		self.canny_pub = rospy.Publisher('/canny_image', Image, queue_size=1)

		# Publisher for output image
		self.image_pub = rospy.Publisher('/output_image', Image, queue_size=1)

	def color_callback(self, color_data):
		try:
			color_image = self.bridge.imgmsg_to_cv2(color_data, "bgr8")
		except CvBridgeError as e:
			print(e)

		gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)

		# canny = cv2.Canny(gray, 50, 255)

		blurred = cv2.GaussianBlur(gray, (5, 5), 0)
		canny = cv2.Canny(blurred, 50, 200)

		kernel = np.ones((3,3), np.uint8)
		dilated_edges = cv2.dilate(canny,kernel, iterations = 1)

		self.canny_pub.publish(self.bridge.cv2_to_imgmsg(dilated_edges, encoding="mono8"))

		contours, hierarchy = cv2.findContours(dilated_edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

		# contours = [cnt for cnt in contours if cv2.contourArea(cnt) > 500]

		for cnt in contours:
			rect = cv2.minAreaRect(cnt)
			box = cv2.boxPoints(rect)
			box = np.intp(box)
			cv2.drawContours(color_image,[box],0,(0,0,255),2)

		# 	angle = rect[2]
		# 	width, height = rect[1]

		# 	print("width:", width, "height:", height, "angle:", angle)

		# print("==================================================")

		# cv2.drawContours(color_image, contours, -1, (0,255,0), 3)

		# contours = [cnt for cnt in contours if cv2.contourArea(cnt) > 500]

		# for cnt in contours:
		#   cv2.drawContours(color_image, [cnt], -1, (0,255,0), 3)

		self.image_pub.publish(self.bridge.cv2_to_imgmsg(color_image, encoding="bgr8"))



if __name__ == '__main__':
	try:
		depth_camera = DepthCamera()
		rospy.spin()
	except KeyboardInterrupt:
		print('Shutting down the ContourFinder Node')
		cv2.destroyAllWindows()