#!/usr/bin/env python3

# Libraries
import rospy
import cv2
import actionlib
import tf2_ros
import tf_conversions
import math
import time

# ROS messages needed
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from opencv_apps.msg import MomentArrayStamped
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from control_msgs.msg import (FollowJointTrajectoryAction, FollowJointTrajectoryGoal)
from control_msgs.msg import (GripperCommandAction, GripperCommandGoal)
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import TransformStamped, PoseStamped, Pose, Point, Quaternion
from moveit_msgs.msg import MoveItErrorCodes
from moveit_python import MoveGroupInterface, PlanningSceneInterface

#Global variables
values = [0.0,0.0,0.0,0.0]
valuesRed = [0.0,0.0,0.0,0.0]
valuesBlue = [0.0,0.0,0.0,0.0]
blue = False
red = False
bridge = CvBridge()
head_moving = False
robotHeight = 1.00
collitionObjectExist = False

head_joint_names = ["head_pan_joint", "head_tilt_joint"]
head_joint_positions = [0.0, 0.0]
torso_joint_name = "torso_lift_joint"
torso_joint_position = 0.0

gripper_closed = 0.03
gripper_open = 0.10

# Checks if a head goal is complete, logs when goal is completed and updates head_moving to False.
def goalDone_cb(state, done):
	rospy.loginfo("Head goal reached")
	global head_moving
	head_moving = False

# Logs when a activity starts running
def active_cb():
	rospy.loginfo("Head movement active")

# Checks if a goal is active then logs the current activity and sets the values og head_moving to True
def feedback_cb(fb):
	rospy.loginfo("Head moving")
	global head_moving
	head_moving = True
	
# Sets head position based on x and y position specified
def head_pos(x,y):
	head_joint_positions
	global head_joint_names
	global head_client
	global head_moving
	
	head_joint_positions[1] = y
	head_joint_positions[0] = x
	
	trajectory = JointTrajectory()
	trajectory.joint_names = head_joint_names
	trajectory.points.append(JointTrajectoryPoint())
	trajectory.points[0].positions = head_joint_positions
	trajectory.points[0].velocities = [0.0]
	trajectory.points[0].accelerations = [0.0]
	trajectory.points[0].time_from_start = rospy.Duration(5.0)

	head_goal = FollowJointTrajectoryGoal()
	head_goal.trajectory = trajectory
	head_goal.goal_time_tolerance = rospy.Duration(0.0)
	
	rospy.loginfo("Setting positions...")
	head_client.send_goal(head_goal, goalDone_cb, active_cb, feedback_cb)
	head_client.wait_for_result(rospy.Duration(6.0))  # specify timeout on waiting
	rospy.loginfo("...done")

	if head_joint_positions[0] > 0.5:
		rospy.loginfo("Object too close to robot")
		lowerRobot()

# Decide if head should look down or up to keep object in frame. Will attempt to keep x unchanged
# NOTE: 0,0 is top left of camera, bottom left is 
def head_pos_center():
	global head_joint_positions
	global values

	x = head_joint_positions[0]
	
	if values[1] < 220:
		rospy.loginfo("Looking up")
		head_joint_positions[1] = head_joint_positions[1] - 0.1
		y = head_joint_positions[1]
	
	elif values[1] > 260:
		rospy.loginfo("Looking down")
		head_joint_positions[1] = head_joint_positions[1] + 0.1
		y = head_joint_positions[1]
		
	else:
		rospy.loginfo("No adjustment needed")
	
	head_pos(x, y)

# Attempts to turn the robot towards the object so allow straight travel
def horizontal_center():
	global values

	if values[0] < 300:
		rospy.loginfo("Object to the right %.2f", values[0])
		return 0.3

	elif values[0] > 340:
		rospy.loginfo("Object to the left %.2f", values[0])
		return -0.3

# Attempts to find the distance (depth) to the target in the image
def cb_depthImage(image):
	global bridge
	global values
	global depth
	
	x = int(values[0])
	y = int(values[1])
	
	try:
		cv_image = bridge.imgmsg_to_cv2(image, "32FC1")
		depth = cv_image[y][x]
		rospy.logdebug('Depth is %s m', depth)
		cv2.circle(cv_image, (x,y), 10, 0)
		cv2.imshow("Image", cv_image)
		cv2.waitKey(3)

		transBroad(depth)
		
	except CvBridgeError as e:
		print(e)
		rospy.loginfo(e)

# Uses the distance to the object and position in image to calculate actual position of object
def transBroad(depth):
	br = tf2_ros.TransformBroadcaster()
	t = TransformStamped()
	global values

	offsetX = -(((values[0] - (640/2))/554.254691191187) * depth) #OffsetX is y
	offsetY = -(((values[1] - (480/2))/554.254691191187) * depth) #OffsetY is z
	
	t.header.stamp = rospy.Time.now()
	t.header.frame_id = "head_camera_depth_frame"
	t.child_frame_id = "target_object"
	t.transform.translation.x = depth
	t.transform.translation.z = offsetY
	t.transform.translation.y = offsetX
	q = tf_conversions.transformations.quaternion_from_euler(0, 0, 0)
	t.transform.rotation.x = q[0]
	t.transform.rotation.y = q[1]
	t.transform.rotation.z = q[2]
	t.transform.rotation.w = q[3]
	
	br.sendTransform(t)

# Grabs information from picture about object location on a 2D plane
def ContourSubBlue(msg):
	global valuesBlue
	global blue
	
	if len(msg.moments) > 0:
		valuesBlue[0] = msg.moments[0].center.x
		valuesBlue[1] = msg.moments[0].center.y
		valuesBlue[2] = msg.moments[0].length
		valuesBlue[3] = msg.moments[0].area
		blue = True
		
	else:
		blue = False

def ContourSubRed(msg):
	global valuesRed
	global red

	if len(msg.moments) > 0:
		valuesRed[0] = msg.moments[0].center.x
		valuesRed[1] = msg.moments[0].center.y
		valuesRed[2] = msg.moments[0].length
		valuesRed[3] = msg.moments[0].area
		red = True
	else:
		red = False

# Pose to use when travelling
# This pose puts the arm away from the camera and raises torso and arm high so it is ready for grabbing the object
def armPrep():
	gripper_poses = Pose(Point(0.042, 0.384, 1.000), Quaternion(0.173, -0.693, -0.242, 0.45))
	moveEndEffector(gripper_poses)

def lowerRobot():
	global robotHeight
	gripper_poses = Pose(Point(0.042, 0.384, x), Quaternion(0.173, -0.693, -0.242, 0.45))
	moveEndEffector(gripper_poses)
	robotHeight += - 0.2

# Calculates pose needed to grab object
def TransListen(objective):
	global tfBuffer
	global listener
	global collitionObjectExist

	planning_scene = PlanningSceneInterface("base_link")

	try:
		trans = tfBuffer.lookup_transform('base_link', 'target_object', rospy.Time())
	except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
		rospy.loginfo("Except")
		#rate.sleep()
		return

	transX = trans.transform.translation.x
	transY = trans.transform.translation.y
	transZ = trans.transform.translation.z

	rotX = trans.transform.rotation.x
	rotY = trans.transform.rotation.y
	rotZ = trans.transform.rotation.z
	rotW = trans.transform.rotation.w

	tableHeight = transZ + -1.13
	topHeight = tableHeight + 2.7
	midHeight = tableHeight + 1.5
	leftSideOffset = transY + -1.3
	rightSideOffset = transY + 1.3

	rotW = 0.45
	rotY = 0.45

	if objective == True:
		if collitionObjectExist:
			PlanningSceneInterface.removeCollisionObject("table")
			PlanningSceneInterface.removeCollisionObject("top")
			PlanningSceneInterface.removeCollisionObject("right")
			PlanningSceneInterface.removeCollisionObject("left")
		planning_scene.addCube("table", 2, 1.20, 0.0, tableHeight)
		planning_scene.addCube("top", 2, 1.75, 0.0, topHeight)
		planning_scene.addCube("right", 2, 1.6, rightSideOffset, midHeight)
		planning_scene.addCube("left", 2, 1.6, leftSideOffset, midHeight)
		collitionObjectExist = True
		transX += 0.10
		transZ += 0.17
	else:
		transZ += 0.5

	gripper_poses = Pose(Point(transX, transY, transZ), Quaternion(rotX, rotY, rotZ, rotW))

	moveEndEffector(gripper_poses)

# Tells the robot to change pose
def moveEndEffector(pose):
	global move_group
	global gripper_pose_stamped
	global gripper_frame

	gripper_pose_stamped.header.stamp = rospy.Time.now()
	gripper_pose_stamped.pose = pose
	move_group.moveToPose(gripper_pose_stamped, gripper_frame)
	result = move_group.get_move_action().get_result()

	if result:
		if result.error_code.val == MoveItErrorCodes.SUCCESS:
			rospy.loginfo("Success")
		else:
			rospy.logerr("Arm goal is in state: %s", move_group.get_move_action().get_state())

	else:
		rospy.logerr("MoveIt! failure. No result returned!")

# Callback function to execute for each time pose message arrives
def poseMessageRecieved(msg):
	global laserValues
	laserValues = msg

# Make gripper open and close
def gripperControl(open):
	global gripper_open
	global gripper_closed
	global gripper_client

	if open:
		gripper_goal = GripperCommandGoal()
		gripper_goal.command.max_effort = 7.0
		gripper_goal.command.position = gripper_open
		rospy.loginfo("Opening gripper...")
	else:
		gripper_goal = GripperCommandGoal()
		gripper_goal.command.max_effort = 7.0
		gripper_goal.command.position = gripper_closed
		rospy.loginfo("Closing gripper...")
	
	gripper_client.send_goal(gripper_goal)
	gripper_client.wait_for_result(rospy.Duration(5.0))

	rospy.loginfo("...done")

# Somehow ended up as the main code to guide the robot
def pubvel():
	global values
	global valuesBlue
	global valuesRed
	global blue
	global red
	global head_client
	global depth
	global reset
	global move_group
	global tfBuffer
	global listener
	global gripper_pose_stamped
	global gripper_frame
	global gripper_client
	global collitionObjectExist

	objectGrabbed = False
	
	# Initialise the ROS system and become a node
	rospy.init_node('Assignment', anonymous=False)
	
	# Create a publisher object
	pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1000)
    
    # Subscribe to contour
	rospy.Subscriber('/contour_moments_blue/moments', MomentArrayStamped, ContourSubBlue)
	rospy.Subscriber("/contour_moments_red/moments", MomentArrayStamped, ContourSubRed)
	rospy.Subscriber("/head_camera/depth_registered/image_raw", Image, cb_depthImage)
	rospy.Subscriber("/base_scan", LaserScan, poseMessageRecieved)

	# Create move group interface for fetch
	move_group = MoveGroupInterface("arm_with_torso", "base_link")

	tfBuffer = tf2_ros.Buffer()
	listener = tf2_ros.TransformListener(tfBuffer)

	gripper_pose_stamped = PoseStamped()
	gripper_pose_stamped.header.frame_id = 'base_link'
	gripper_frame = "wrist_roll_link"
	
	# Define ground plane
	planning_scene = PlanningSceneInterface("base_link")
	planning_scene.removeCollisionObject("my_front_ground")
	planning_scene.removeCollisionObject("my_back_ground")
	planning_scene.removeCollisionObject("my_right_ground")
	planning_scene.removeCollisionObject("my_left_ground")
	planning_scene.addCube("my_front_ground", 2, 1.1, 0.0, -1.0)
	planning_scene.addCube("my_back_ground", 2, -1.2, 0.0, -1.0)
	planning_scene.addCube("my_left_ground", 2, 0.0, 1.2, -1.0)
	planning_scene.addCube("my_right_ground", 2, 0.0, -1.2, -1.0)

	# Loop at 10Hz until the node is shutdown
	rate = rospy.Rate(10)
	
	rospy.loginfo("Waiting for head_controller...")
	head_client = actionlib.SimpleActionClient("head_controller/follow_joint_trajectory", FollowJointTrajectoryAction)
	head_client.wait_for_server()
	rospy.loginfo("...connected.")

	rospy.loginfo("Waiting for gripper_controller...")
	gripper_client = actionlib.SimpleActionClient("gripper_controller/gripper_action", GripperCommandAction)
	gripper_client.wait_for_server()
	rospy.loginfo("...connected")

	# Reset head position to 0,0
	head_pos(0, 0)

	# Makes sure the gripper is open and that anything that might be held is dropped
	gripperControl(True)

	# Move arm out of the way of the camera and rais it to prepare for grabbing
	armPrep()
	
	while not rospy.is_shutdown():
		msg = Twist()
		
		if objectGrabbed == False:
			values = valuesBlue
		else:
			values = valuesRed

		if blue or red:
			rospy.loginfo("Object found")
			msg.angular.z = 0
			msg.linear.x = 0
			pub.publish(msg)
			
			rospy.loginfo("Locking on object")
			
			rospy.loginfo("X pos  = %.2f",values[0])
			rospy.loginfo("Y pos  = %.2f",values[1])
			rospy.loginfo("Length = %.2f",values[2])
			rospy.loginfo("Area   = %.2f",values[3])
			rospy.loginfo("Depth  = %.2f",depth)
			
			if depth > 0.7:
				while depth > 0.7:
					if values[0] < 300 or values[0] > 340:
						rospy.loginfo("Target is to the side, correcting")
						while values[0] < 300 or values[0] > 340:
							msg.angular.z = horizontal_center()
							msg.linear.x = 0.1
							pub.publish(msg)
						msg.angular.z = 0.0
						msg.linear.x = 0.0
						pub.publish(msg)
					
					elif values[1] > 380 or values[1] < 100:
						rospy.loginfo("Object not in center. Correcting")
						msg.linear.x = 0
						pub.publish(msg)
						head_pos_center()
						
					else:
						rospy.loginfo("Target in center of frame, approaching")
						rospy.loginfo("Approaching")
						msg.linear.x = 0.3
						pub.publish(msg)
			
			elif depth < 0.7 and objectGrabbed == False:
				if values[1] > 380 or values[1] < 100:
					head_pos_center()
				rospy.loginfo("Reached object. Depth = %.2f",depth)
				rospy.loginfo("Navigating arm")
				gripperControl(True)
				TransListen(True)
				i = 0
				while i < 17:
					msg.linear.x = -0.1
					pub.publish(msg)
					rate.sleep()
					i += 1
				msg.linear.x = 0.0
				pub.publish(msg)
				gripperControl(False)
				objectGrabbed = True
				armPrep()
				head_pos(0, 0)

			elif depth < 0.8 and objectGrabbed == True:
				msg.linear.x = 0
				pub.publish(msg)
				if values[1] > 380 or values[1] < 100:
					head_pos_center()
				rospy.loginfo("Dropping object")
				rospy.loginfo("Navigating arm")
				TransListen(False)
				gripperControl(True)
				objectGrabbed = False
				armPrep()
				head_pos(0, 0)

		# Search for any object until one is within range
		else:
			rospy.loginfo("Object not found")
			msg.linear.x = -0.1
			while not blue:
				msg.angular.z = 0.5
				pub.publish(msg)
				rate.sleep()
			msg.angular.z = 0.0
			pub.publish(msg)
		
		# Wait until it's time for another iteration
		rate.sleep()


if __name__ == '__main__':
	try:
		pubvel()
	except rospy.ROSInterruptException:
		pass
