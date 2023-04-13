#!/usr/bin/env python3
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import tf2_ros
import tf2_geometry_msgs
import math
import numpy as np
import time
from math import pi
from std_msgs.msg import UInt16, Int8
from geometry_msgs.msg import Pose, PoseStamped, TransformStamped
from detection_msgs.msg import Detection2DArray, Detection2D
from moveit_commander.conversions import pose_to_list
from tf.transformations import quaternion_from_euler

class TransformPose:
	def __init__(self):
		#initialise moveit_commander and a rospy node
		moveit_commander.roscpp_initialize(sys.argv)
		rospy.init_node('PickPlace_Node', anonymous=True)

		#instantiate a RobotCommander object
		robot = moveit_commander.RobotCommander()

		#instantitate a PlanningSceneInterface object
		scene = moveit_commander.PlanningSceneInterface()

		#instantitate a MoveGroupCommander object
		self.arm_group = moveit_commander.MoveGroupCommander("manipulator")

		self.arm_group.set_num_planning_attempts(20)

		#Create DisplayTrajectory ROS publisher which is used to display trajectories in Rviz
		display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)

		self.tf_buffer = tf2_ros.Buffer()
		tf_listener = tf2_ros.TransformListener(self.tf_buffer)

		#Subscribe to object pose
		self.object_pose = rospy.Subscriber('/yolov5/detections', Detection2DArray, self.detection_callback)

		#Subscribe to yellow plate pose
		self.yellow_plate_pose_sub = rospy.Subscriber('/pose_list/yellow', Detection2DArray, self.yellow_pos_callback)

		#Subscribe to green plate pose
		self.green_plate_pose_sub = rospy.Subscriber('/pose_list/green', Detection2DArray, self.green_pos_callback)

		#Publish to rotate servos
		self.servo_orientate_pub = rospy.Publisher('/change_mode', Int8, queue_size=1)

		#Publish to pressurise gripper
		self.valve_pub = rospy.Publisher('/valve', UInt16, queue_size=1)

		self.mask_activate_pub = rospy.Publisher('/mask_activate', UInt16, queue_size=1)

	def Plate_TransformPose(self, plate_data_array, plate_list):

		for detection in plate_data_array.detections:

			plate_pose = Detection2D()
			plate_pose.header.stamp = detection.header.stamp
			plate_pose.header.frame_id = detection.header.frame_id

			#Unused in the code
			plate_pose.bbox = detection.bbox 
			plate_pose.results.score = detection.results.score

			plate_pose.results.Class = detection.results.Class
			# plate_pose.results.pose = detection.results.pose

			plate_pose_stamped = PoseStamped()
			plate_pose_stamped.header.stamp = plate_pose.header.stamp
			plate_pose_stamped.header.frame_id = plate_pose.header.frame_id

			plate_pose_stamped.pose = detection.results.pose

			try:
				camera_to_robot_transform_stamped = self.tf_buffer.lookup_transform("world", "camera_color_optical_frame", rospy.Time())
				plate_to_base_pose = tf2_geometry_msgs.do_transform_pose(plate_pose_stamped, camera_to_robot_transform_stamped)

				plate_pose.results.pose = plate_to_base_pose.pose

				plate_list.append(plate_pose)

				# print("Received object pose wrt to robot base", plate_pose)
				# print(camera_to_robot_transform_stamped)
			except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
				rospy.logerr("Failed to transform object pose from camera frame to robot's frame")

		return plate_list

	def yellow_pos_callback(self, plate_data):

		yellow_plate_pose_list = []

		yellow_plate_pose_list = self.Plate_TransformPose(plate_data, yellow_plate_pose_list)

		self.yellow_plate_pose_list = yellow_plate_pose_list

	def green_pos_callback(self, plate_data):

		green_plate_pose_list = []

		green_plate_pose_list = self.Plate_TransformPose(plate_data, green_plate_pose_list)

		self.green_plate_pose_list = green_plate_pose_list

	def detection_callback(self, Detection2DArray_data):

		container_pose_list = []
		detected_obj_pose_list = []
		
		for detection in Detection2DArray_data.detections:

			detected_obj = Detection2D()
			detected_obj.header.stamp = detection.header.stamp
			detected_obj.header.frame_id = detection.header.frame_id
			
			detected_obj.bbox = detection.bbox

			detected_obj.results.Class = detection.results.Class
			detected_obj.results.score = detection.results.score

			# detected_obj.results.pose = detection.results.pose

			detected_pose_stamped = PoseStamped()
			detected_pose_stamped.header.stamp = detected_obj.header.stamp
			detected_pose_stamped.header.frame_id = detected_obj.header.frame_id

			# detected_pose_stamped.pose = detected_obj.results.pose
			detected_pose_stamped.pose = detection.results.pose

			try:
				camera_to_robot_transform_stamped = self.tf_buffer.lookup_transform("world", "camera_color_optical_frame", rospy.Time())
				object_pose = tf2_geometry_msgs.do_transform_pose(detected_pose_stamped, camera_to_robot_transform_stamped)

				detected_obj.results.pose = object_pose.pose

				if detected_obj.results.Class == "containers":
					container_pose_list.append(detected_obj)
				else:
					detected_obj_pose_list.append(detected_obj)

				# print("Received object pose wrt to robot base", plate_pose)
				# print(camera_to_robot_transform_stamped)
			except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
				rospy.logerr("Failed to transform object pose from camera frame to robot's frame")

		self.container_pose_list = container_pose_list
		self.detected_obj_pose_list = detected_obj_pose_list

class PickPlace:
	def __init__(self):
		self.tp = TransformPose()

	def HomePosition(self):
		print(">> Going to Home Position")
		self.tp.arm_group.set_named_target("HomePosition")
		self.tp.arm_group.go(wait=True)

		rospy.sleep(1)

	def AssemblyTable(self):
		#Move above assembly table
		print(">> Going above assembly table")
		assembly = [1.44,-1.48, 0.83885,-0.91, -pi/2, 1.44069]
		self.tp.arm_group.set_joint_value_target(assembly)
		self.tp.arm_group.go(wait=True)

		rospy.sleep(1)

	def AssemblyTable_Bottle(self):
		#Move above assembly table
		print(">> Going above assembly table")
		assembly = [-1.41982,-1.611, 1.32, 0.264, 1.320, 0]
		self.tp.arm_group.set_joint_value_target(assembly)
		self.tp.arm_group.go(wait=True)

		rospy.sleep(1)

	def Mean_XYZ(self, pose_list):
		pose_results = [pose.results.pose for pose in pose_list]

		x_list = [pose.position.x for pose in pose_results]
		y_list = [pose.position.y for pose in pose_results]
		z_list = [pose.position.z for pose in pose_results]

		x_mean = np.mean(x_list)
		y_mean = np.mean(y_list)
		z_mean = np.mean(z_list)

		mean_pose = np.array([x_mean, y_mean, z_mean])

		return mean_pose

	def SortContainers(self):

		yellow_plate_container_pose_list = []
		green_plate_container_pose_list = []

		yellow_plate_container_pose_list = self.tp.yellow_plate_pose_list
		green_plate_container_pose_list = self.tp.green_plate_pose_list

		yellow_average = self.Mean_XYZ(self.tp.yellow_plate_pose_list)
		green_average = self.Mean_XYZ(self.tp.green_plate_pose_list)

		for container in self.tp.container_pose_list:
			container_x = container.results.pose.position.x
			container_y = container.results.pose.position.y
			container_z = container.results.pose.position.z

			boundingbox_x = int(container.bbox.size_x)
			boundingbox_y = int(container.bbox.size_y)

			boundingbox_area = boundingbox_x * boundingbox_y

			if boundingbox_area < 1500:
				continue
			elif boundingbox_area < 1800 and boundingbox_y < 45:
				container.results.Class = "small_container"
			elif boundingbox_area < 1900 and boundingbox_y > 45:
				container.results.Class = "rect_container"
			else:
				container.results.Class = "big_container"

			container_pos = np.array([container_x, container_y, container_z])

			yellow_distance = np.linalg.norm(container_pos - yellow_average)
			green_distance = np.linalg.norm(container_pos - green_average)

			if yellow_distance < green_distance:
				yellow_plate_container_pose_list.append(container)
			else:
				green_plate_container_pose_list.append(container)

		self.yellow_plate_container_pose_list = yellow_plate_container_pose_list
		self.green_plate_container_pose_list = green_plate_container_pose_list

		print(">> Poses on Yellow Tray")
		print(self.yellow_plate_container_pose_list)
		print(">> Poses on Green Tray")
		print(self.green_plate_container_pose_list)

	# def AssemblyTable_SortPosition(self):

	# 	print(">> Sleeping for 5 seconds for YOLO to catch up")
	# 	rospy.sleep(5)

	# 	self.SortContainers()

	# 	yellow_container_visit_count = {}
	# 	green_container_visit_count = {}

	# 	for pose in self.yellow_plate_container_pose_list:
	# 		x_pos = pose.results.pose.position.x
	# 		y_pos = pose.results.pose.position.y
	# 		z_pos = pose.results.pose.position.z

	# 		if pose.results.Class == "container":
	# 			yellow_container_visit_count[(x_pos, y_pos, z_pos)] = 0

	# 	for pose in self.green_plate_container_pose_list:
	# 		x_pos = pose.results.pose.position.x
	# 		y_pos = pose.results.pose.position.y
	# 		z_pos = pose.results.pose.position.z

	# 		if pose.results.Class == "container":
	# 			green_container_visit_count[(x_pos, y_pos, z_pos)] = 0

	# 	self.yellow_container_visit_count = yellow_container_visit_count
	# 	self.green_container_visit_count = green_container_visit_count

	def MoveToPose(self, pose):
		self.tp.arm_group.set_pose_target(pose)
		self.tp.arm_group.go(wait=True)

		rospy.sleep(1)

	def MoveToJoint(self, joint):
		self.tp.arm_group.set_joint_value_target(joint)
		self.tp.arm_group.go(wait=True)

		rospy.sleep(1)

	def PickBottle(self, obj):

		source_object = geometry_msgs.msg.Pose()
		
		source_object.position.x = obj.results.pose.position.x + 0.17
		source_object.position.y = obj.results.pose.position.y #Depends on world reference
		source_object.position.z = 1.3

		gripper_orientation = quaternion_from_euler(pi, -pi/2, 0)

		source_object.orientation.x = gripper_orientation[0]
		source_object.orientation.y = gripper_orientation[1]
		source_object.orientation.z = gripper_orientation[2]
		source_object.orientation.w = gripper_orientation[3]

		print(">> Going above detected object:", obj.results.Class)
		print(source_object)
		
		self.MoveToPose(source_object)

		before_pick = self.tp.arm_group.get_current_joint_values()

		pickpose = self.tp.arm_group.get_current_pose().pose

		pickpose.position.z = 0.8 #Adjust according to the height
		
		self.MoveToPose(pickpose)

		self.GripperValve('inflate')

		#Move upwards in z-direction
		print(">> Moving upwards along the z-axis")
		pickpose = self.tp.arm_group.get_current_pose().pose
		pickpose.position.z = 1.3

		self.MoveToPose(pickpose)

		return source_object, before_pick

	def SourceTable_Object_Low(self, obj):

		#Go to above object on source table (Replace with camera)
		source_object = geometry_msgs.msg.Pose()
		
		source_object.position.x = obj.results.pose.position.x
		source_object.position.y = obj.results.pose.position.y
		source_object.position.z = 1.1

		self.source_object_depth = obj.results.pose.position.z

		perp_orientation = quaternion_from_euler(pi,0,pi/2)

		source_object.orientation.x = perp_orientation[0]
		source_object.orientation.y = perp_orientation[1]
		source_object.orientation.z = perp_orientation[2]
		source_object.orientation.w = perp_orientation[3]

		rospy.sleep(1)

		print(">> Going above detected object:", obj.results.Class)
		print(source_object)
		
		self.MoveToPose(source_object)

		rospy.sleep(1)

	def SourceTable_Object_High(self, obj):

		#Go to above object on source table (Replace with camera)
		source_object = geometry_msgs.msg.Pose()
		
		source_object.position.x = obj.results.pose.position.x
		source_object.position.y = obj.results.pose.position.y
		source_object.position.z = 1.44

		self.source_object_depth = obj.results.pose.position.z

		perp_orientation = quaternion_from_euler(pi,0, pi)

		source_object.orientation.x = perp_orientation[0]
		source_object.orientation.y = perp_orientation[1]
		source_object.orientation.z = perp_orientation[2]
		source_object.orientation.w = perp_orientation[3]

		rospy.sleep(1)

		print(">> Going above detected object:", obj.results.Class)
		print(source_object)
		
		self.MoveToPose(source_object)

		rospy.sleep(1)


	def GripperValve(self, state):

		msg = UInt16()

		if state == 'inflate':
			print(">> Pressuring gripper, picking up item")
			msg.data = 1

		elif state == 'deflate':
			print(">> Depressuring gripper, placing item")
			msg.data = 0 
		
		else:
			print('>> GripperValve: Unknown string detected, unable to assign value')

		self.tp.valve_pub.publish(msg)

		rospy.sleep(1)


	def Pick_Low(self, depth, obj):

		#Move towards object in z-direction
		pickpose = self.tp.arm_group.get_current_pose().pose

		print(">> Moving downwards along the z-axis")
						
		if obj == 'cookies':
			pickpose.position.z = depth + 0.195 #Adjust according to the height
		elif obj == 'fried_eggs':
			pickpose.position.z = depth + 0.185
		else:
			pickpose.position.z = depth + 0.175
		
		self.MoveToPose(pickpose)

		rospy.sleep(1)

		if obj == 'broccoli' or obj == 'carrot_slice' or obj == 'cookies' or obj == 'fried_eggs' or obj == 'iced_gems' or obj == 'meatballs':

			orientation = "Perpendicular"
			self.ServoOrientation(orientation)

		elif obj == 'Green_Beans' or obj == 'Sausage' or obj == 'bottle' or obj == 'noodles':

			orientation = "Parallel"
			self.ServoOrientation(orientation)

		self.GripperValve('inflate')

		#Move upwards in z-direction
		print(">> Moving upwards along the z-axis")
		pickpose = self.tp.arm_group.get_current_pose().pose
		pickpose.position.z = 1.1

		self.MoveToPose(pickpose)

	def Pick_High(self, depth, obj):

		#Move towards object in z-direction
		pickpose = self.tp.arm_group.get_current_pose().pose

		print(">> Moving downwards along the z-axis")
		
		if obj == 'carrot_slice':				
			pickpose.position.z = depth + 0.20 #Adjust according to the height
		elif obj == 'broccoli':
			pickpose.position.z = depth + 0.185
		else:
			pickpose.position.z = depth + 0.20

		print(pickpose)

		self.MoveToPose(pickpose)

		rospy.sleep(1)

		if obj == 'broccoli' or obj == 'carrot_slice' or obj == 'cookies' or obj == 'fried_eggs' or obj == 'iced_gems' or obj == 'meatballs':

			orientation = "Perpendicular"
			self.ServoOrientation(orientation)

		elif obj == 'Green_Beans' or obj == 'Sausage' or obj == 'bottle' or obj == 'noodles':

			orientation = "Parallel"
			self.ServoOrientation(orientation)

		self.GripperValve('inflate')

		#Move upwards in z-direction
		print(">> Moving upwards along the z-axis")
		pickpose = self.tp.arm_group.get_current_pose().pose
		pickpose.position.z = 1.47

		self.MoveToPose(pickpose)

	def Place(self):
		#Move towards object in z-direction
		print(">> Moving 0.15 meters downpwards along the z-axis")
		placepose = self.tp.arm_group.get_current_pose().pose
		placepose.position.z = 0.74+0.025+0.23
		
		self.MoveToPose(placepose)

		self.GripperValve('deflate')

		#Move upwards in z-direction
		print(">> Moving 0.15 meters upwards along the z-axis")
		placepose = self.tp.arm_group.get_current_pose().pose
		placepose.position.z = 1.3
		
		self.MoveToPose(placepose)

		rospy.sleep(1)

	def ServoOrientation(self, orientation):
		print(">> Servo Orientation set to", orientation)
		msg = Int8()

		if orientation == 'Perpendicular':
			msg.data = 0
		elif orientation == 'Parallel':
			msg.data = 1
		else:
			print('>> ServoOrientation: Unknown string detected, unable to assign value')

		self.tp.servo_orientate_pub.publish(msg)

	def Pour(self):

		placepose = self.tp.arm_group.get_current_pose().pose
		placepose.position.z = 0.9

		self.MoveToPose(placepose)

		pouring_angle = 3*pi/4

		pour_joint_angles = self.tp.arm_group.get_current_joint_values()
		pour_joint_angles[-1] -= pouring_angle

		print(">> Beginning pour")
		self.MoveToJoint(pour_joint_angles)

		print(">> Holding for 8 seconds")
		rospy.sleep(8)

		pour_joint_angles[-1] += pouring_angle

		print(">> Stopping pour")
		self.MoveToJoint(pour_joint_angles)

		placepose = self.tp.arm_group.get_current_pose().pose
		placepose.position.z = 1.3
		
		self.MoveToPose(placepose)
	
	def PlaceBottle(self, original_pose):
		
		self.MoveToPose(original_pose)

		print(">> Moving downwards along the z-axis")
		pickpose = self.tp.arm_group.get_current_pose().pose
		pickpose.position.z = 0.8 #Adjust according to the height
		
		self.MoveToPose(pickpose)

		self.GripperValve('deflate')

		#Move upwards in z-direction
		print(">> Moving upwards along the z-axis")
		pickpose = self.tp.arm_group.get_current_pose().pose
		pickpose.position.z = 1.3

		self.MoveToPose(pickpose)	

	def PnP_Location_Low(self, plate_list, pick_count, place_count, location):

		print(plate_list)

		waypoint = [0.59562, -1.91429, 2.21425, -1.9025, -1.56555, 3.63817]
		print(">> Moving to waypoint")
		self.MoveToJoint(waypoint)

		print(">> Moving to overview point")
		self.MoveToJoint(location)

		print(">> Sleeping for 5 seconds for YOLO to catch up")
		rospy.sleep(5)
	
		start_time = time.time()

		while any(place_count[obj] < pick_count[obj] for obj in pick_count):
			for obj in pick_count:
				while place_count[obj] < pick_count[obj]:
					
					end_time = time.time()
					elapsed_time = end_time - start_time

					if elapsed_time > 120 and place_count[obj] == 0:
						return

					for detected_obj in self.tp.detected_obj_pose_list: 
						if detected_obj.results.Class == obj:
							
							self.ServoOrientation("Parallel")

							self.SourceTable_Object_Low(detected_obj)

							# if obj == 'bottle':

							# 	original_bottle = self.PickBottle(detected_obj)
							# 	self.AssemblyTable_Bottle()

							# else:

							# 	self.SourceTable_Object_Low(detected_obj, orientation)
							# 	self.Pick_Low(self.source_object_depth, obj) 
							# 	self.AssemblyTable()
									
							self.SourceTable_Object_Low(detected_obj)
							self.Pick_Low(self.source_object_depth, obj) 
							
							print(">> Moving to waypoint")
							self.MoveToJoint(waypoint)

							self.AssemblyTable()

							for pose in plate_list:
								x_pos = pose.results.pose.position.x
								y_pos = pose.results.pose.position.y
								z_pos = pose.results.pose.position.z

								if obj == 'noodles' or obj == 'meatballs' or obj == 'Sausage':
									if pose.results.Class == 'plate':

										plate_arm_pose = geometry_msgs.msg.Pose()

										plate_arm_pose.position.x = x_pos
										plate_arm_pose.position.y = y_pos
										plate_arm_pose.position.z = 1.3

										quartenion = quaternion_from_euler(0, pi, 0)
										plate_arm_pose.orientation.x = quartenion[0]
										plate_arm_pose.orientation.y = quartenion[1]
										plate_arm_pose.orientation.z = quartenion[2]
										plate_arm_pose.orientation.w = quartenion[3]

										print(">>Going above plate")

										break

								# elif obj == 'broccoli' or obj == 'carrot_slice':
								# 	if pose.results.Class == 'bowl':

								# 		plate_arm_pose = geometry_msgs.msg.Pose()

								# 		plate_arm_pose.position.x = x_pos
								# 		plate_arm_pose.position.y = y_pos
								# 		plate_arm_pose.position.z = 1.3

								# 		quartenion = quaternion_from_euler(0, pi, 0)
								# 		plate_arm_pose.orientation.x = quartenion[0]
								# 		plate_arm_pose.orientation.y = quartenion[1]
								# 		plate_arm_pose.orientation.z = quartenion[2]
								# 		plate_arm_pose.orientation.w = quartenion[3]

								# 		print(">>Going above bowl")

								# 		break

								elif obj == 'fried_eggs':
									if pose.results.Class == 'big_container':

										plate_arm_pose = geometry_msgs.msg.Pose()

										plate_arm_pose.position.x = x_pos
										plate_arm_pose.position.y = y_pos
										plate_arm_pose.position.z = 1.3

										quartenion = quaternion_from_euler(0, pi, 0)
										plate_arm_pose.orientation.x = quartenion[0]
										plate_arm_pose.orientation.y = quartenion[1]
										plate_arm_pose.orientation.z = quartenion[2]
										plate_arm_pose.orientation.w = quartenion[3]

										# container_visit_count[(x_pos, y_pos, z_pos)] += 1

										print(">>Going above big container")

										break

								elif obj == 'cookies':
									if pose.results.Class == 'rect_container':

										plate_arm_pose = geometry_msgs.msg.Pose()

										plate_arm_pose.position.x = x_pos
										plate_arm_pose.position.y = y_pos
										plate_arm_pose.position.z = 1.3

										quartenion = quaternion_from_euler(0, pi, 0)
										plate_arm_pose.orientation.x = quartenion[0]
										plate_arm_pose.orientation.y = quartenion[1]
										plate_arm_pose.orientation.z = quartenion[2]
										plate_arm_pose.orientation.w = quartenion[3]

										# container_visit_count[(x_pos, y_pos, z_pos)] += 1

										print(">>Going above big container")

										break

								# elif obj == 'bottle':
								# 	if pose.results.Class == 'cup':

								# 		plate_arm_orientation = self.tp.arm_group.get_current_pose().pose

								# 		plate_arm_pose = geometry_msgs.msg.Pose()

								# 		plate_arm_pose.position.x = x_pos - 0.10
								# 		plate_arm_pose.position.y = y_pos + 0.17
								# 		plate_arm_pose.position.z = 1.3

								# 		plate_arm_pose.orientation.x = plate_arm_orientation.orientation.x
								# 		plate_arm_pose.orientation.y = plate_arm_orientation.orientation.y
								# 		plate_arm_pose.orientation.z = plate_arm_orientation.orientation.z
								# 		plate_arm_pose.orientation.w = plate_arm_orientation.orientation.w

								# 		print(">>Going above plate")

								# 		break


							print(plate_arm_pose)

							self.MoveToPose(plate_arm_pose)
							
							# if obj == 'bottle':
							# 	self.Pour()

							# 	self.MoveToJoint(before_pick)

							# 	self.PlaceBottle(original_bottle)

							# else:
							# 	self.Place()

							self.Place()

							place_count[obj] += 1
							rospy.sleep(1)

							print(">> Moving to waypoint")
							self.MoveToJoint(waypoint)

							self.MoveToJoint(location)
							
							print(">> Sleeping for 5 seconds for YOLO to catch up")
							rospy.sleep(5)	

							break							

				else:
					print("All ", obj, "has been placed")

		print(">> All objects from the given location have been placed")

	def PnP_Location_High(self, plate_list, pick_count, place_count, location):

		print(plate_list)

		print(">> Moving to overview point")
		self.MoveToJoint(location)

		print(">> Sleeping for 5 seconds for YOLO to catch up")
		rospy.sleep(5)

		start_time = time.time()
	
		while any(place_count[obj] < pick_count[obj] for obj in pick_count):
			for obj in pick_count:
				while place_count[obj] < pick_count[obj]:
					
					end_time = time.time()
					elapsed_time = end_time - start_time

					if elapsed_time > 120 and place_count[obj] == 0:
						return

					for detected_obj in self.tp.detected_obj_pose_list: 
						if detected_obj.results.Class == obj:

							self.ServoOrientation("Parallel")

							self.SourceTable_Object_High(detected_obj)

							self.Pick_High(self.source_object_depth, obj) 
							
							overview = [0.61599, -1.4584, 1.00819, -1.16042, -1.59096, 1.6564]
							self.MoveToJoint(overview)

							self.AssemblyTable()

							for pose in plate_list:
								x_pos = pose.results.pose.position.x
								y_pos = pose.results.pose.position.y
								z_pos = pose.results.pose.position.z

								if obj == 'meatballs' or obj == 'Sausage':
									if pose.results.Class == 'plate':

										plate_arm_pose = geometry_msgs.msg.Pose()

										plate_arm_pose.position.x = x_pos
										plate_arm_pose.position.y = y_pos
										plate_arm_pose.position.z = 1.3

										quartenion = quaternion_from_euler(0, pi, 0)
										plate_arm_pose.orientation.x = quartenion[0]
										plate_arm_pose.orientation.y = quartenion[1]
										plate_arm_pose.orientation.z = quartenion[2]
										plate_arm_pose.orientation.w = quartenion[3]

										print(">>Going above plate")

										break

								elif obj == 'broccoli' or obj == 'carrot_slice':
									if pose.results.Class == 'bowl':

										plate_arm_pose = geometry_msgs.msg.Pose()

										plate_arm_pose.position.x = x_pos
										plate_arm_pose.position.y = y_pos
										plate_arm_pose.position.z = 1.3

										quartenion = quaternion_from_euler(0, pi, 0)
										plate_arm_pose.orientation.x = quartenion[0]
										plate_arm_pose.orientation.y = quartenion[1]
										plate_arm_pose.orientation.z = quartenion[2]
										plate_arm_pose.orientation.w = quartenion[3]

										print(">>Going above bowl")

										break

								# elif obj == 'fried_eggs':
								# 	if pose.results.Class == 'big_container':

								# 		plate_arm_pose = geometry_msgs.msg.Pose()

								# 		plate_arm_pose.position.x = x_pos
								# 		plate_arm_pose.position.y = y_pos
								# 		plate_arm_pose.position.z = 1.3

								# 		quartenion = quaternion_from_euler(0, pi, 0)
								# 		plate_arm_pose.orientation.x = quartenion[0]
								# 		plate_arm_pose.orientation.y = quartenion[1]
								# 		plate_arm_pose.orientation.z = quartenion[2]
								# 		plate_arm_pose.orientation.w = quartenion[3]

								# 		# container_visit_count[(x_pos, y_pos, z_pos)] += 1

								# 		print(">>Going above big container")

								# 		break

								# elif obj == 'Cookies' or obj == 'iced_gems':
								# 	if pose.results.Class == 'small_container':

								# 		plate_arm_pose = geometry_msgs.msg.Pose()

								# 		plate_arm_pose.position.x = x_pos
								# 		plate_arm_pose.position.y = y_pos
								# 		plate_arm_pose.position.z = 1.3

								# 		quartenion = quaternion_from_euler(0, pi, 0)
								# 		plate_arm_pose.orientation.x = quartenion[0]
								# 		plate_arm_pose.orientation.y = quartenion[1]
								# 		plate_arm_pose.orientation.z = quartenion[2]
								# 		plate_arm_pose.orientation.w = quartenion[3]

								# 		# container_visit_count[(x_pos, y_pos, z_pos)] += 1

								# 		print(">>Going above big container")

								# 		break

								# elif obj == 'Cookies' or obj == 'iced_gems':
								# 	if pose.results.Class == 'rect_container':

								# 		plate_arm_pose = geometry_msgs.msg.Pose()

								# 		plate_arm_pose.position.x = x_pos
								# 		plate_arm_pose.position.y = y_pos
								# 		plate_arm_pose.position.z = 1.3

								# 		quartenion = quaternion_from_euler(0, pi, 0)
								# 		plate_arm_pose.orientation.x = quartenion[0]
								# 		plate_arm_pose.orientation.y = quartenion[1]
								# 		plate_arm_pose.orientation.z = quartenion[2]
								# 		plate_arm_pose.orientation.w = quartenion[3]

								# 		# container_visit_count[(x_pos, y_pos, z_pos)] += 1

								# 		print(">>Going above big container")

								# 		break

							print(plate_arm_pose)

							self.MoveToPose(plate_arm_pose)
							
							self.Place()

							place_count[obj] += 1
							rospy.sleep(1)

							self.MoveToJoint(overview)

							self.MoveToJoint(location)
							
							print(">> Sleeping for 5 seconds for YOLO to catch up")
							rospy.sleep(5)	

							break							

				else:
					print("All ", obj, "has been placed")

		print(">> All objects from the given location have been placed")

	def PnP_Location_Own(self, plate_list, pick_count, place_count, location):

		print(plate_list)

		waypoint = [0.59562, -1.91429, 2.21425, -1.9025, -1.56555, 3.63817]
		print(">> Moving to waypoint")
		self.MoveToJoint(waypoint)

		print(">> Moving to overview point")
		self.MoveToJoint(location)

		print(">> Sleeping for 5 seconds for YOLO to catch up")
		rospy.sleep(5)
	
		start_time = time.time()

		while any(place_count[obj] < pick_count[obj] for obj in pick_count):
			for obj in pick_count:
				while place_count[obj] < pick_count[obj]:
					
					end_time = time.time()
					elapsed_time = end_time - start_time

					if elapsed_time > 120 and place_count[obj] == 0:
						return

					for detected_obj in self.tp.detected_obj_pose_list: 
						if detected_obj.results.Class == obj:
							
							self.ServoOrientation("Parallel")

							self.SourceTable_Object_Low(detected_obj)

							pickpose = self.tp.arm_group.get_current_pose().pose

							print(">> Moving downwards along the z-axis")
											
							pickpose.position.z = 0.96
							
							self.MoveToPose(pickpose)

							rospy.sleep(1)

							if obj == 'broccoli' or obj == 'carrot_slice' or obj == 'cookies' or obj == 'fried_eggs' or obj == 'iced_gems' or obj == 'meatballs':

								orientation = "Perpendicular"
								self.ServoOrientation(orientation)

							elif obj == 'Green_Beans' or obj == 'Sausage' or obj == 'bottle' or obj == 'noodles':

								orientation = "Parallel"
								self.ServoOrientation(orientation)

							self.GripperValve('inflate')

							#Move upwards in z-direction
							print(">> Moving upwards along the z-axis")
							pickpose = self.tp.arm_group.get_current_pose().pose
							pickpose.position.z = 1.1

							self.MoveToPose(pickpose) 
							
							print(">> Moving to waypoint")
							self.MoveToJoint(waypoint)
							
							self.AssemblyTable()

							for pose in plate_list:
								x_pos = pose.results.pose.position.x
								y_pos = pose.results.pose.position.y
								z_pos = pose.results.pose.position.z


								if obj == 'iced_gems':
									if pose.results.Class == 'small_container':

										plate_arm_pose = geometry_msgs.msg.Pose()

										plate_arm_pose.position.x = x_pos
										plate_arm_pose.position.y = y_pos
										plate_arm_pose.position.z = 1.3

										quartenion = quaternion_from_euler(0, pi, 0)
										plate_arm_pose.orientation.x = quartenion[0]
										plate_arm_pose.orientation.y = quartenion[1]
										plate_arm_pose.orientation.z = quartenion[2]
										plate_arm_pose.orientation.w = quartenion[3]

										# container_visit_count[(x_pos, y_pos, z_pos)] += 1

										print(">>Going above rectangle container")

										break

							print(plate_arm_pose)

							self.MoveToPose(plate_arm_pose)
							
							self.Place()

							place_count[obj] += 1
							rospy.sleep(1)

							self.MoveToJoint(overview)

							self.MoveToJoint(location)
							
							print(">> Sleeping for 5 seconds for YOLO to catch up")
							rospy.sleep(5)	

							break							

				else:
					print("All ", obj, "has been placed")

		print(">> All objects from the given location have been placed")

	def Green_Beans(self, plate_list, pick_count, place_count, location):

		self.MoveToJoint(location)

		while any(place_count[obj] < pick_count[obj] for obj in pick_count):
			for obj in pick_count:
				while place_count[obj] < pick_count[obj]:

					orientation = "Parallel"
					self.ServoOrientation(orientation)

					green_bean_joint = [0.47956, -1.421, 2.1518, -2.30621, -1.59451, 3.5444]

					# green_bean_pose.position.x = 0.373
					# green_bean_pose.position.y = 0.332
					# green_bean_pose.position.z = 1.0724

					# green_bean_pose.orientation.x = 0.699
					# green_bean_pose.orientation.y = 0.714
					# green_bean_pose.orientation.z = 0
					# green_bean_pose.orientation.w = 0

					self.MoveToJoint(green_bean_joint)

					#Pick
					green_bean_pose = self.tp.arm_group.get_current_pose().pose
					green_bean_pose.position.z = 0.95

					self.MoveToPose(green_bean_pose)

					self.GripperValve('inflate')

					green_bean_pose.position.z = 1.0724

					self.MoveToPose(green_bean_pose)

					self.AssemblyTable()

					for pose in plate_list:
						x_pos = pose.results.pose.position.x
						y_pos = pose.results.pose.position.y
						z_pos = pose.results.pose.position.z

						if pose.results.Class == 'bowl':

							plate_arm_pose = geometry_msgs.msg.Pose()

							plate_arm_pose.position.x = x_pos
							plate_arm_pose.position.y = y_pos
							plate_arm_pose.position.z = 1.3

							quartenion = quaternion_from_euler(0, pi, 0)
							plate_arm_pose.orientation.x = quartenion[0]
							plate_arm_pose.orientation.y = quartenion[1]
							plate_arm_pose.orientation.z = quartenion[2]
							plate_arm_pose.orientation.w = quartenion[3]

							print(">>Going above bowl")

							break

					print(plate_arm_pose)

					self.MoveToPose(plate_arm_pose)
					
					self.Place()

					place_count[obj] += 1
					rospy.sleep(1)

					self.MoveToJoint(location)
						
					print(">> Sleeping for 5 seconds for YOLO to catch up")
					rospy.sleep(5)	

					break							

			else:
				print("All ", obj, "has been placed")

			print(">> All objects from the given location have been placed")

	def Noodles(self, plate_list, pick_count, place_count, location):

		self.MoveToJoint(location)

		while any(place_count[obj] < pick_count[obj] for obj in pick_count):
			for obj in pick_count:
				while place_count[obj] < pick_count[obj]:

					orientation = "Parallel"
					self.ServoOrientation(orientation)

					noodle_joint = [0.07841, -1.68356, 2.27887, -2.1644, -1.57207, 3.24997]

					self.MoveToJoint(noodle_joint)

					#Pick
					noodle_pose = self.tp.arm_group.get_current_pose().pose
					noodle_pose.position.z = 0.96

					self.MoveToPose(noodle_pose)

					self.GripperValve('inflate')

					noodle_pose.position.z = 1.09

					self.MoveToPose(noodle_pose)

					self.AssemblyTable()

					for pose in plate_list:
						x_pos = pose.results.pose.position.x
						y_pos = pose.results.pose.position.y
						z_pos = pose.results.pose.position.z

						if pose.results.Class == 'plate':

							plate_arm_pose = geometry_msgs.msg.Pose()

							plate_arm_pose.position.x = x_pos
							plate_arm_pose.position.y = y_pos
							plate_arm_pose.position.z = 1.3

							quartenion = quaternion_from_euler(0, pi, 0)
							plate_arm_pose.orientation.x = quartenion[0]
							plate_arm_pose.orientation.y = quartenion[1]
							plate_arm_pose.orientation.z = quartenion[2]
							plate_arm_pose.orientation.w = quartenion[3]

							print(">>Going above bowl")

							break

					print(plate_arm_pose)

					self.MoveToPose(plate_arm_pose)
					
					self.Place()

					place_count[obj] += 1
					rospy.sleep(1)

					self.MoveToJoint(location)
						
					print(">> Sleeping for 5 seconds for YOLO to catch up")
					rospy.sleep(5)	

					break							

			else:
				print("All ", obj, "has been placed")

			print(">> All objects from the given location have been placed")	



if __name__ == '__main__':
	pickplace_object = PickPlace()
	
	# pickplace_object.HomePosition()

	pickplace_object.AssemblyTable()
	pickplace_object.SortContainers()

	msg = UInt16()
	msg.data = 1

	pickplace_object.tp.mask_activate_pub.publish(msg)
	
	yellow_plate_container_pose_list = pickplace_object.yellow_plate_container_pose_list
	green_plate_container_pose_list = pickplace_object.green_plate_container_pose_list

	location_1 = [0.66175, -1.31804, 1.98216, -2.24336, -pi/2, 3.77591] #Green Beans
	location_2 = [0.26269, -1.61863, 2.2372, -2.28861, -pi/2, 3.37671] #Noodles
	location_3 = [-0.18071, -1.78405, 2.46317, -2.25836, -pi/2, 2.93352] #cookies
	location_4 = [-0.658851, -1.75588, 2.44318, -2.26378, -pi/2, 2.4556] #Fried eggs

	location_5 = [0.11245, -1.45687, 1.00274, -1.15654, -1.59093, 1.6564] #Broccoli
	location_6 = [-0.38188, -1.53847, 1.12277, -1.19964, -1.56965, 1.16274] #Carrot

	overview = [0.61599, -1.4584, 1.00819, -1.16042, -1.59096, 1.6564]

	# #Test pick
	# pick_count_1 = {'Green_Beans': 1}
	# pick_count_2 = {'noodles': 1}
	# pick_count_3 = {'cookies': 1, 'carrot_slice': 1} #Cookie mistaken as carrot_slice
	# pick_count_4 = {'fried_eggs': 1} 

	# pick_count_5 = {'broccoli': 1}
	# pick_count_6 = {'carrot_slice': 1}
	# pick_count_7 = {'broccoli': 1}

	#Actual pick
	pick_count_1 = {'iced_gems': 5}
	pick_count_2 = {'noodles': 3}
	pick_count_3 = {'cookies': 2, 'carrot_slice': 1} #Cookie mistaken as bottle
	pick_count_4 = {'fried_eggs': 1} 

	pick_count_5 = {'broccoli': 5}
	pick_count_6 = {'carrot_slice': 5}
	# pick_count_7 = {'broccoli': 3}

	yellow_place_count_1 = {'iced_gems': 0}
	yellow_place_count_2 = {'noodles': 0}
	yellow_place_count_3 = {'cookies': 0, 'carrot_slice': 0} #Cookie mistaken as bottle
	yellow_place_count_4 = {'fried_eggs': 0} 

	yellow_place_count_5 = {'broccoli': 0}
	yellow_place_count_6 = {'carrot_slice': 0}
	# yellow_place_count_7 = {'broccoli': 0}

	green_place_count_1 = {'iced_gems': 0}
	green_place_count_2 = {'noodles': 0}
	green_place_count_3 = {'cookies': 0, 'carrot_slice': 0} #Cookie mistaken as bottle
	green_place_count_4 = {'fried_eggs': 0} 

	green_place_count_5 = {'broccoli': 0}
	green_place_count_6 = {'carrot_slice': 0}
	# green_place_count_7 = {'broccoli': 0}

	# current_pose = pickplace_object.tp.arm_group.get_current_pose()
	# print(current_pose)

	pickplace_object.PnP_Location_Own(yellow_plate_container_pose_list, pick_count_1, yellow_place_count_1, location_1)
	pickplace_object.Noodles(yellow_plate_container_pose_list, pick_count_2, yellow_place_count_2, location_2)
	pickplace_object.PnP_Location_Low(yellow_plate_container_pose_list, pick_count_3, yellow_place_count_3, location_3)
	pickplace_object.PnP_Location_Low(yellow_plate_container_pose_list, pick_count_4, yellow_place_count_4, location_4) 

	elevate = pickplace_object.tp.arm_group.get_current_pose().pose
	elevate.position.z = 1.5

	pickplace_object.MoveToPose(elevate)

	pickplace_object.PnP_Location_High(yellow_plate_container_pose_list, pick_count_5, yellow_place_count_5, location_5)
	pickplace_object.PnP_Location_High(yellow_plate_container_pose_list, pick_count_6, yellow_place_count_6, location_6)

	pickplace_object.MoveToJoint(overview)

	pickplace_object.PnP_Location_Own(green_plate_container_pose_list, pick_count_1, green_place_count_1, location_1)
	pickplace_object.Noodles(green_plate_container_pose_list, pick_count_2, green_place_count_2, location_2)
	pickplace_object.PnP_Location_Low(green_plate_container_pose_list, pick_count_3, green_place_count_3, location_3)
	pickplace_object.PnP_Location_Low(green_plate_container_pose_list, pick_count_4, green_place_count_4, location_4) 

	elevate = pickplace_object.tp.arm_group.get_current_pose().pose
	elevate.position.z = 1.5

	pickplace_object.MoveToPose(elevate)

	pickplace_object.PnP_Location_High(green_plate_container_pose_list, pick_count_5, green_place_count_5, location_5)
	pickplace_object.PnP_Location_High(green_plate_container_pose_list, pick_count_6, green_place_count_6, location_6)

	pickplace_object.AssemblyTable()

	#Assemble Yellow Tray
	# pickplace_object.PnP_Location_Low(yellow_plate_container_pose_list, pick_count_1, yellow_place_count_1, location_1)

	# pickplace_object.Green_Beans(yellow_plate_container_pose_list, pick_count_1, yellow_place_count_1, location_1)
	# pickplace_object.Noodles(yellow_plate_container_pose_list, pick_count_2, yellow_place_count_2, location_2)
	# pickplace_object.PnP_Location_Low(yellow_plate_container_pose_list, pick_count_3, yellow_place_count_3, location_3)
	# pickplace_object.PnP_Location_Low(yellow_plate_container_pose_list, pick_count_4, yellow_place_count_4, location_4) #Egg too big

	# elevate = pickplace_object.tp.arm_group.get_current_pose().pose
	# elevate.position.z = 1.5

	# pickplace_object.MoveToPose(elevate)

	# pickplace_object.PnP_Location_High(yellow_plate_container_pose_list, pick_count_5, yellow_place_count_5, location_5)
	# pickplace_object.PnP_Location_High(yellow_plate_container_pose_list, pick_count_6, yellow_place_count_6, location_6)
	# pickplace_object.PnP_Location_Own(yellow_plate_container_pose_list, pick_count_7, yellow_place_count_7, location_5)

	# pickplace_object.MoveToJoint(overview)

	# #Assemble Green Tray
	# # pickplace_object.PnP_Location_Low(green_plate_container_pose_list, pick_count_1, green_place_count_1, location_1)

	# pickplace_object.Green_Beans(green_plate_container_pose_list, pick_count_1, green_place_count_1, location_1)
	# pickplace_object.Noodles(green_plate_container_pose_list, pick_count_2, green_place_count_2, location_2)
	# pickplace_object.PnP_Location_Low(green_plate_container_pose_list, pick_count_3, green_place_count_3, location_3)
	# pickplace_object.PnP_Location_Low(green_plate_container_pose_list, pick_count_4, green_place_count_4, location_4)

	# elevate = pickplace_object.tp.arm_group.get_current_pose().pose
	# elevate.position.z = 1.5

	# pickplace_object.MoveToPose(elevate)

	# pickplace_object.PnP_Location_High(green_plate_container_pose_list, pick_count_5, green_place_count_5, location_5)
	# pickplace_object.PnP_Location_High(green_plate_container_pose_list, pick_count_6, green_place_count_6, location_6)
	# pickplace_object.PnP_Location_Own(green_plate_container_pose_list, pick_count_7, green_place_count_7, location_5)

	pickplace_object.AssemblyTable()

	# location_1 = [0, -2.0361, 1.84368, -1.3466 ,-pi/2, pi/2] ##Front left
	# location_2 = [-0.74742, -2.07635, 1.86412, -1.33707, -1.59464, 0.82166] ##Front right
	# location_3 = [0.18684, -1.25406, 0.57442, -0.9251 ,-pi/2, 1.75163] ##Top left
	# location_4 = [-0.2942, -1.39396, 0.7431, -0.96597 ,-pi/2, 1.27078] ##Top right
	# # location_5 = [-1.4127, -1.76352, 1.56914, -1.40462, -1.55953. 0.15767] ## Bottle

	# pick_count_1 = {'Green_Beans': 1, 'noodles': 1}
	# pick_count_2 = {'meatballs': 1} #Actual cookie
	# pick_count_3 = {'broccoli': 1}
	# pick_count_4 = {'carrot_slice': 1}

	# yellow_place_count_1 = {'Green_Beans': 0, 'noodles': 0}
	# yellow_place_count_2 = {'meatballs': 0} #Actual cookie
	# yellow_place_count_3 = {'broccoli': 0}
	# yellow_place_count_4 = {'carrot_slice': 0}

	# green_place_count_1 = {'Green_Beans': 0}
	# # green_place_count_2 = {'red_cube': 0, 'blue_cube': 0}
	# # green_place_count_3 = {'red_cube': 0, 'blue_cube': 0}

	# # Assemble Tray 1 which has yellow plates on it
	# # pickplace_object.PnP_Location_Low(yellow_plate_container_pose_list, pick_count_1, yellow_place_count_1, location_1)
	# pickplace_object.PnP_Location_Low(yellow_plate_container_pose_list, pick_count_2, yellow_place_count_2, location_2)
	# pickplace_object.PnP_Location_High(yellow_plate_container_pose_list, pick_count_3, yellow_place_count_3, location_3)
	# pickplace_object.PnP_Location_High(yellow_plate_container_pose_list, pick_count_4, yellow_place_count_4, location_4)

	# # Assemble Tray 2 which has blue plates on it
	# # pickplace_object.PnP_Location(green_plate_container_pose_list, pick_count_1, green_place_count_1, location_1)
	# # pickplace_object.PnP_Location(blue_plate_container_pose_list, pick_count_2, blue_place_count_2, location_2)
	# # pickplace_object.PnP_Location(blue_plate_container_pose_list, pick_count_3, blue_place_count_3, location_3)

	# pickplace_object.AssemblyTable()
