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

		# #Subscribe to red plate pose
		# self.red_plate_pose_sub = rospy.Subscriber('/pose_list/red', Detection2DArray, self.red_pos_callback)

		# #Subscribe to blue plate pose
		# self.blue_plate_pose_sub = rospy.Subscriber('/pose_list/blue', Detection2DArray, self.blue_pos_callback)

		#Publish to rotate servos
		self.servo_orientate_pub = rospy.Publisher('/change_mode', Int8, queue_size=1)

		#Publish to pressurise gripper
		self.valve_pub = rospy.Publisher('/valve', UInt16, queue_size=1)

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

	def red_pos_callback(self, plate_data):

		red_plate_pose_list = []

		red_plate_pose_list = self.Plate_TransformPose(plate_data, red_plate_pose_list)

		self.red_plate_pose_list = red_plate_pose_list

	def blue_pos_callback(self, plate_data):

		blue_plate_pose_list = []

		blue_plate_pose_list = self.Plate_TransformPose(plate_data, blue_plate_pose_list)

		self.blue_plate_pose_list = blue_plate_pose_list

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

				if detected_obj.results.Class == "green_cube":
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
		assembly = [-pi/2,-pi/2, 1.466,-1.431, -pi/2, 0]
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

	# def OrientationConstraint(self):
	# 	orientation_constraint = moveit_msgs.msg.OrientationConstraint()
	# 	orientation_constraint.header.frame_id = "base_link"
	# 	orientation_constraint.link_name = "wrist_3_link"
	# 	orientation_constraint.orientation = geometry_msgs.msg.Quaternion(w=-1, x=0, y=0, z=0)
	# 	orientation_constraint.absolute_x_axis_tolerance = 0.1
	# 	orientation_constraint.absolute_y_axis_tolerance = 0.1
	# 	orientation_constraint.absolute_z_axis_tolerance = 0.05
	# 	orientation_constraint.weight = 1.0

	# 	path_constraints = moveit_msgs.msg.Constraints()
	# 	path_constraints.orientation_constraints.append(orientation_constraint)
	# 	self.tp.arm_group.set_path_constraints(path_constraints)

	def SortContainers(self):

		red_plate_container_pose_list = []
		blue_plate_container_pose_list = []

		red_plate_container_pose_list = self.tp.red_plate_pose_list
		blue_plate_container_pose_list = self.tp.blue_plate_pose_list

		red_average = self.Mean_XYZ(self.tp.red_plate_pose_list)
		blue_average = self.Mean_XYZ(self.tp.blue_plate_pose_list)

		for container in self.tp.container_pose_list:
			container_x = container.results.pose.position.x
			container_y = container.results.pose.position.y
			container_z = container.results.pose.position.z

			container_pos = np.array([container_x, container_y, container_z])

			red_distance = np.linalg.norm(container_pos - red_average)
			blue_distance = np.linalg.norm(container_pos - blue_average)

			if red_distance < blue_distance:
				red_plate_container_pose_list.append(container)
			else:
				blue_plate_container_pose_list.append(container)

		self.red_plate_container_pose_list = red_plate_container_pose_list
		self.blue_plate_container_pose_list = blue_plate_container_pose_list

		print(">> Poses on Red Tray")
		print(self.red_plate_container_pose_list)
		print(">> Poses on Blue Tray")
		print(self.blue_plate_container_pose_list)

	def AssemblyTable_SortPosition(self):

		print(">> Sleeping for 25 seconds for YOLO to catch up")
		rospy.sleep(25)

		self.SortContainers()

		red_container_visit_count = {}
		blue_container_visit_count = {}

		for pose in self.red_plate_container_pose_list:
			x_pos = pose.results.pose.position.x
			y_pos = pose.results.pose.position.y
			z_pos = pose.results.pose.position.z

			if pose.results.Class == "green_cube":
				red_container_visit_count[(x_pos, y_pos, z_pos)] = 0

		for pose in self.blue_plate_container_pose_list:
			x_pos = pose.results.pose.position.x
			y_pos = pose.results.pose.position.y
			z_pos = pose.results.pose.position.z

			if pose.results.Class == "green_cube":
				blue_container_visit_count[(x_pos, y_pos, z_pos)] = 0

		self.red_container_visit_count = red_container_visit_count
		self.blue_container_visit_count = blue_container_visit_count

	def MoveToPose(self, pose):
		self.tp.arm_group.set_pose_target(pose)
		self.tp.arm_group.go(wait=True)

		rospy.sleep(1)

	def MoveToJoint(self, joint):
		self.tp.arm_group.set_joint_value_target(joint)
		self.tp.arm_group.go(wait=True)

		rospy.sleep(1)

	def SourceTable_Object(self, obj, orientation):

		#Go to above object on source table (Replace with camera)
		source_object = geometry_msgs.msg.Pose()
		
		source_object.position.x = obj.results.pose.position.x
		source_object.position.y = obj.results.pose.position.y
		source_object.position.z = 1.3

		self.source_object_depth = obj.results.pose.position.z

		if orientation == 'Perpendicular':
			source_object.orientation.x = (obj.results.pose.orientation.x)
			source_object.orientation.y = (obj.results.pose.orientation.y)
			source_object.orientation.z = (obj.results.pose.orientation.z)
			source_object.orientation.w = (obj.results.pose.orientation.w)
		
		elif orientation == 'Parallel':
			
			perp_orientation = quaternion_from_euler(0,pi,-pi)

			source_object.orientation.x = perp_orientation[0]
			source_object.orientation.y = perp_orientation[1]
			source_object.orientation.z = perp_orientation[2]
			source_object.orientation.w = perp_orientation[3]

		rospy.sleep(1)

		#Always printing blue_cube
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


	def Pick(self, depth):

		# ######################################
		# ### Replace with air pressure node ###
		# ######################################
		# deflate = [0.087, 0.087, 0.087, 0.087] #5degrees

		# print(">> Deflate gripper")
		# self.tp.hand_group.set_joint_value_target(deflate)
		# self.tp.hand_group.go()
		# ### Replace with air pressure node ###

		##### INSERT Deflate gripper code #######

		#Move towards object in z-direction
		pickpose = self.tp.arm_group.get_current_pose().pose

		print(">> Moving downwards along the z-axis")

		pickpose.position.z = depth + 0.17 + 0.025 #Adjust according to the height
		
		self.MoveToPose(pickpose)

		rospy.sleep(1)

		self.GripperValve('inflate')

		#Move upwards in z-direction
		print(">> Moving upwards along the z-axis")
		pickpose = self.tp.arm_group.get_current_pose().pose
		pickpose.position.z = 1.4

		self.MoveToPose(pickpose)


	def Place(self):
		#Move towards object in z-direction
		print(">> Moving 0.15 meters downpwards along the z-axis")
		placepose = self.tp.arm_group.get_current_pose().pose
		placepose.position.z -= 0.15
		
		self.MoveToPose(placepose)

		self.GripperValve('deflate')

		#Move upwards in z-direction
		print(">> Moving 0.15 meters upwards along the z-axis")
		placepose = self.tp.arm_group.get_current_pose().pose
		placepose.position.z += 0.15
		
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

	def PnP_Location(self, plate_list, container_visit_count, pick_count, place_count, location):

		print(plate_list)

		print(">> Moving to overview point")
		self.MoveToJoint(location)

		print(">> Sleeping for 5 seconds for YOLO to catch up")
		rospy.sleep(5)
	
		while any(place_count[obj] < pick_count[obj] for obj in pick_count):
			for obj in pick_count:
				while place_count[obj] < pick_count[obj]:
					for detected_obj in self.tp.detected_obj_pose_list: 
						if detected_obj.results.Class == obj:
							if obj == 'red_cube':

								orientation = "Perpendicular"
								self.ServoOrientation(orientation)

							elif obj == 'blue_cube':

								orientation = "Parallel"
								self.ServoOrientation(orientation)


							self.SourceTable_Object(detected_obj)
							self.Pick(self.source_object_depth) 
							self.AssemblyTable()

							for pose in plate_list:
								x_pos = pose.results.pose.position.x
								y_pos = pose.results.pose.position.y
								z_pos = pose.results.pose.position.z

							
								if obj == 'red_cube':

									if pose.results.Class == 'plate':

										plate_arm_pose = geometry_msgs.msg.Pose()

										plate_arm_pose.position.x = x_pos
										plate_arm_pose.position.y = y_pos
										plate_arm_pose.position.z = 1.1

										quartenion = quaternion_from_euler(0, pi, 0)
										plate_arm_pose.orientation.x = quartenion[0]
										plate_arm_pose.orientation.y = quartenion[1]
										plate_arm_pose.orientation.z = quartenion[2]
										plate_arm_pose.orientation.w = quartenion[3]

										print(">>Going above plate")

										break

								elif obj == 'blue_cube':

									if pose.results.Class == 'green_cube' and container_visit_count[(x_pos, y_pos, z_pos)] == 0:

										plate_arm_pose = geometry_msgs.msg.Pose()

										plate_arm_pose.position.x = x_pos
										plate_arm_pose.position.y = y_pos
										plate_arm_pose.position.z = 1.1

										quartenion = quaternion_from_euler(0, pi, 0)
										plate_arm_pose.orientation.x = quartenion[0]
										plate_arm_pose.orientation.y = quartenion[1]
										plate_arm_pose.orientation.z = quartenion[2]
										plate_arm_pose.orientation.w = quartenion[3]

										container_visit_count[(x_pos, y_pos, z_pos)] += 1

										print(">>Going above container")

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
	
	# # pickplace_object.HomePosition()

	# pickplace_object.AssemblyTable()
	# pickplace_object.AssemblyTable_SortPosition()
	
	# red_plate_container_pose_list = pickplace_object.red_plate_container_pose_list
	# blue_plate_container_pose_list = pickplace_object.blue_plate_container_pose_list

	# red_container_visit_count = pickplace_object.red_container_visit_count
	# blue_container_visit_count = pickplace_object.blue_container_visit_count

	# location_1 = [-3.752, -1.431, 0.960, -1.1 ,-pi/2, -0.593]
	# # location_2 = [-3.752, -1.431, 0.960, -1.1 ,-pi/2, -0.593]
	# # location_3 = [-3.752, -1.431, 0.960, -1.1 ,-pi/2, -0.593]

	# pick_count_1 = {'red_cube': 2, 'blue_cube': 1}
	# # pick_count_2 = {'red_cube': 2, 'blue_cube': 1}
	# # pick_count_3 = {'red_cube': 2, 'blue_cube': 1}

	# red_place_count_1 = {'red_cube': 0, 'blue_cube': 0}
	# # red_place_count_2 = {'red_cube': 0, 'blue_cube': 0}
	# # red_place_count_3 = {'red_cube': 0, 'blue_cube': 0}

	# blue_place_count_1 = {'red_cube': 0, 'blue_cube': 0}
	# # blue_place_count_2 = {'red_cube': 0, 'blue_cube': 0}
	# # blue_place_count_3 = {'red_cube': 0, 'blue_cube': 0}

	# # Assemble Tray 1 which has red plates on it
	# pickplace_object.PnP_Location(red_plate_container_pose_list, red_container_visit_count, pick_count_1, red_place_count_1, location_1)
	# # pickplace_object.PnP_Location(red_plate_container_pose_list, red_container_visit_count, pick_count_2, red_place_count_2, location_2)
	# # pickplace_object.PnP_Location(red_plate_container_pose_list, red_container_visit_count, pick_count_3, red_place_count_3, location_3)

	# # Assemble Tray 2 which has blue plates on it
	# pickplace_object.PnP_Location(blue_plate_container_pose_list, blue_container_visit_count, pick_count_1, blue_place_count_1, location_1)
	# # pickplace_object.PnP_Location(blue_plate_container_pose_list, blue_container_visit_count, pick_count_2, blue_place_count_2, location_2)
	# # pickplace_object.PnP_Location(blue_plate_container_pose_list, blue_container_visit_count, pick_count_3, blue_place_count_3, location_3)

	# pickplace_object.HomePosition()


	overview_location = [pi/2, -pi/2, 1.414, -1.414, -pi/2, pi/2]
	pickplace_object.MoveToJoint(overview_location)

	print(">> Sleeping for 5 seconds")
	rospy.sleep(5)

	# pickplace_object.OrientationConstraint()

	for detected_obj in pickplace_object.tp.detected_obj_pose_list: 
		if detected_obj.results.Class == 'broccoli':
			orientation = "Perpendicular"

		elif detected_obj.results.Class == 'Green_Beans':
			orientation = "Parallel"

		pickplace_object.ServoOrientation(orientation)
		pickplace_object.SourceTable_Object(detected_obj, orientation)
		pickplace_object.Pick(pickplace_object.source_object_depth) 

		print(">> Sleeping for 10 seconds")
		rospy.sleep(10)

		pickplace_object.MoveToJoint(overview_location)

		pickplace_object.Place()		

		break

	print(">> ENDING")
