#!/usr/bin/env python3

import rospy
import cv2
import torch
import torch.backends.cudnn as cudnn
import numpy as np
from cv_bridge import CvBridge
from pathlib import Path
import os
import sys
from rostopic import get_topic_type

import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import Image, CompressedImage, CameraInfo, PointCloud2
from detection_msgs.msg import Detection2DArray, Detection2D
from geometry_msgs.msg import Pose
from tf.transformations import quaternion_from_matrix, quaternion_from_euler, quaternion_multiply, quaternion_about_axis, rotation_matrix

from math import pi
from sklearn.decomposition import PCA
from sklearn.preprocessing import StandardScaler

# add yolov5 submodule to path
FILE = Path(__file__).resolve()
ROOT = FILE.parents[0] / "yolov5"
if str(ROOT) not in sys.path:
	sys.path.append(str(ROOT))  # add ROOT to PATH
ROOT = Path(os.path.relpath(ROOT, Path.cwd()))  # relative path

# import from yolov5 submodules
from models.common import DetectMultiBackend
from utils.general import (
	check_img_size,
	check_requirements,
	non_max_suppression,
	scale_coords
)
from utils.plots import Annotator, colors
from utils.torch_utils import select_device
from utils.augmentations import letterbox


@torch.no_grad()
class Yolov5Detector:
	def __init__(self):
		self.conf_thres = rospy.get_param("~confidence_threshold")
		self.iou_thres = rospy.get_param("~iou_threshold")
		self.agnostic_nms = rospy.get_param("~agnostic_nms")
		self.max_det = rospy.get_param("~maximum_detections")
		self.classes = rospy.get_param("~classes", None)
		self.line_thickness = rospy.get_param("~line_thickness")
		self.view_image = rospy.get_param("~view_image")
		# Initialize weights 
		weights = rospy.get_param("~weights")
		# Initialize model
		self.device = select_device(str(rospy.get_param("~device","")))
		self.model = DetectMultiBackend(weights, device=self.device, dnn=rospy.get_param("~dnn"), data=rospy.get_param("~data"))
		self.stride, self.names, self.pt, self.jit, self.onnx, self.engine = (
			self.model.stride,
			self.model.names,
			self.model.pt,
			self.model.jit,
			self.model.onnx,
			self.model.engine,
		)

		# Setting inference size
		self.img_size = [rospy.get_param("~inference_size_w", 640), rospy.get_param("~inference_size_h",480)]
		self.img_size = check_img_size(self.img_size, s=self.stride)

		# Half
		self.half = rospy.get_param("~half", False)
		self.half &= (
			self.pt or self.jit or self.onnx or self.engine
		) and self.device.type != "cpu"  # FP16 supported on limited backends with CUDA
		if self.pt or self.jit:
			self.model.model.half() if self.half else self.model.model.float()
		bs = 1  # batch_size
		cudnn.benchmark = True  # set True to speed up constant image size inference
		self.model.warmup()  # warmup        
		
		# Initialize CV_Bridge
		self.bridge = CvBridge()

		# Subscriber for the depth image
		self.depth_sub = rospy.Subscriber('/camera/aligned_depth_to_color/image_raw', Image, self.depth_callback)

		# Subscriber for depth camera's intrinsic parameters
		camera_info = rospy.wait_for_message('/camera/aligned_depth_to_color/camera_info', CameraInfo)
		self.fx = camera_info.K[0]
		self.fy = camera_info.K[4]
		self.pp_cx = camera_info.K[2]
		self.pp_cy = camera_info.K[5]


		# Initialize subscriber to Image/CompressedImage topic
		input_image_type, input_image_topic, _ = get_topic_type(rospy.get_param("~input_image_topic"), blocking = True)
		self.compressed_input = input_image_type == "sensor_msgs/CompressedImage"

		if self.compressed_input:
			self.image_sub = rospy.Subscriber(
				input_image_topic, CompressedImage, self.callback, queue_size=1
			)
		else:
			self.image_sub = rospy.Subscriber(
				input_image_topic, Image, self.callback, queue_size=1
			)

		# Initialize prediction publisher
		# self.pred_pub = rospy.Publisher(
		#     rospy.get_param("~output_topic"), BoundingBoxes, queue_size=10
		# )

		# Publishing Detection2DArray
		self.pred_pub = rospy.Publisher(
			rospy.get_param("~output_topic"), Detection2DArray, queue_size=10
		)

		# Initialize image publisher
		self.publish_image = rospy.get_param("~publish_image")
		if self.publish_image:
			self.image_pub = rospy.Publisher(
				rospy.get_param("~output_image_topic"), Image, queue_size=10
			)

	def callback(self, data):
		"""adapted from yolov5/detect.py"""
		# print(data.header)
		if self.compressed_input:
			im = self.bridge.compressed_imgmsg_to_cv2(data, desired_encoding="bgr8")
		else:
			im = self.bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
		
		im, im0 = self.preprocess(im)

		# Run inference
		im = torch.from_numpy(im).to(self.device) 
		im = im.half() if self.half else im.float()
		im /= 255
		if len(im.shape) == 3:
			im = im[None]

		pred = self.model(im, augment=False, visualize=False)
		pred = non_max_suppression(
			pred, self.conf_thres, self.iou_thres, self.classes, self.agnostic_nms, max_det=self.max_det
		)

		### To-do move pred to CPU and fill BoundingBox messages
		
		# Process predictions 
		det = pred[0].cpu().numpy()

		# bounding_boxes = BoundingBoxes()
		# bounding_boxes.header = data.header
		# bounding_boxes.image_header = data.header

		detection_array = Detection2DArray()
		detection_array.header.stamp = rospy.Time.now()
		detection_array.header.frame_id = "camera_color_optical_frame"
		
		annotator = Annotator(im0, line_width=self.line_thickness, example=str(self.names))
		if len(det):
			# Rescale boxes from img_size to im0 size
			det[:, :4] = scale_coords(im.shape[2:], det[:, :4], im0.shape).round()

			# Write results
			for *xyxy, conf, cls in reversed(det):
				# bounding_box = BoundingBox()
				# c = int(cls)
				# # Fill in bounding box message
				# bounding_box.Class = self.names[c]
				# bounding_box.probability = conf 
				# bounding_box.xmin = int(xyxy[0])
				# bounding_box.ymin = int(xyxy[1])
				# bounding_box.xmax = int(xyxy[2])
				# bounding_box.ymax = int(xyxy[3])

				# bounding_boxes.bounding_boxes.append(bounding_box)

				detected_obj = Detection2D()
				detected_obj.header.stamp = rospy.Time.now()
				detected_obj.header.frame_id = "camera_depth_optical_frame"

				xmin = xyxy[0]
				ymin = xyxy[1]
				xmax = xyxy[2]
				ymax = xyxy[3]

				cx = float((xmin + xmax)/2)
				cy = float((ymin + ymax)/2)

				detected_obj.bbox.center.x = cx
				detected_obj.bbox.center.y = cy
				detected_obj.bbox.center.theta = 0

				detected_obj.bbox.size_x =  float((xmax-xmin)/2)
				detected_obj.bbox.size_y = float((ymax-ymin)/2)

				depth = self.depth_image[int(cy), int(cx)]/1000

				x_meters = (cx - self.pp_cx) * depth / self.fx
				y_meters = (cy - self.pp_cy) * depth / self.fy
				z_meters = depth

				c = int(cls)
				detected_obj.results.Class = self.names[c]
				detected_obj.results.score = float(conf)

				obj_pose = Pose()

				obj_pose.position.x = x_meters
				obj_pose.position.y = y_meters
				obj_pose.position.z = z_meters

				xmin_meters = (xmin - self.pp_cx) * depth / self.fx
				ymin_meters = (ymin - self.pp_cy) * depth / self.fy

				xmax_meters = (xmax - self.pp_cx) * depth / self.fx
				ymax_meters = (ymax - self.pp_cy) * depth / self.fy

				#Segment the object from the background using the bounding box found by YOLOv5
				
				ymin_expand = ymin-5
				xmin_expand = xmin-5

				ymax_expand = ymax+5
				xmax_expand = xmax+5

				if xmin_expand < 0:
					xmin_expand = 0

				if ymin_expand < 0:
					ymin_expand = 0

				if xmax_expand > 640:
					xmax_expand = 640

				if ymax_expand > 480:
					ymax_expand = 480

				roi = im0[int(ymin_expand):int(ymax_expand), int(xmin_expand):int(xmax_expand)]

				gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)

				blurred = cv2.GaussianBlur(gray, (5, 5), 0)

				canny = cv2.Canny(blurred, 50, 200)

				kernel = np.ones((3,3), np.uint8)
				dilated_edges = cv2.dilate(canny,kernel, iterations = 1)

				contours, hierarchy = cv2.findContours(dilated_edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

				longest_contour_length = 0

				for cnt in contours:
					contour_length = len(cnt)
					if contour_length > longest_contour_length:
						longest_contour_length = longest_contour_length
						longest_contour = cnt

				rect = cv2.minAreaRect(longest_contour)				
				angle = rect[-1]

				width, height = rect[1]
			
				if height > width:
					angle += 90

				angle -= 90

				print("Angle for", detected_obj.results.Class, "@ (x:", cx, "y:", cy, ") :", angle)				

				angle_radians = np.radians(angle)

				orientation_quaternion = quaternion_from_euler(0, 0, angle_radians)

				obj_pose.orientation.x = orientation_quaternion[0]
				obj_pose.orientation.y = orientation_quaternion[1]
				obj_pose.orientation.z = orientation_quaternion[2]
				obj_pose.orientation.w = orientation_quaternion[3]

				detected_obj.results.pose = obj_pose

				detection_array.detections.append(detected_obj)

				# Annotate the image
				if self.publish_image or self.view_image:  # Add bbox to image
					  # integer class
					label = f"{self.names[c]} {conf:.2f}"
					annotator.box_label(xyxy, label, color=colors(c, True))   
					cv2.circle(im0, (int(cx), int(cy)), 4, (0, 255, 0), -1)    

				
				### POPULATE THE DETECTION MESSAGE HERE
			print("=============================================================")

			# Stream results
			im0 = annotator.result()

		# Publish prediction
		# self.pred_pub.publish(bounding_boxes)
		self.pred_pub.publish(detection_array)

		# Publish & visualize images
		if self.view_image:
			cv2.imshow(str(0), im0)
			cv2.waitKey(1)  # 1 millisecond
		if self.publish_image:
			self.image_pub.publish(self.bridge.cv2_to_imgmsg(im0, "bgr8"))
		

	def preprocess(self, img):
		"""
		Adapted from yolov5/utils/datasets.py LoadStreams class
		"""
		img0 = img.copy()
		img = np.array([letterbox(img, self.img_size, stride=self.stride, auto=self.pt)[0]])
		# Convert
		img = img[..., ::-1].transpose((0, 3, 1, 2))  # BGR to RGB, BHWC to BCHW
		img = np.ascontiguousarray(img)

		return img, img0 

	def depth_callback(self,depth_data):
		
		try:
			depth_image = self.bridge.imgmsg_to_cv2(depth_data, "32FC1")
		except CvBridgeError as e:
			print(e)

		self.depth_image = depth_image

if __name__ == "__main__":

	check_requirements(exclude=("tensorboard", "thop"))
	
	rospy.init_node("yolov5", anonymous=True)
	detector = Yolov5Detector()
	
	rospy.spin()
