#!/usr/bin/env python3

import rospy
import cv2
import actionlib
import tf2_ros
import tf_conversions

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from opencv_apps.msg import MomentArrayStamped
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from control_msgs.msg import (FollowJointTrajectoryAction,
                              FollowJointTrajectoryGoal)
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import TransformStamped

#Global variables
values = [0.0,0.0,0.0,0.0]
green = False
bridge = CvBridge()
head_moving = False

head_joint_names = ["head_pan_joint", "head_tilt_joint"]
head_joint_positions = [0.0, 0.0]

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

def horizontal_center():
	global values

	if values[0] < 300:
		rospy.loginfo("Object to the right %.2f", values[0])
		return 0.3

	elif values[0] > 340:
		rospy.loginfo("Object to the left %.2f", values[0])
		return -0.3

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
		
		transBroad()
		
	except CvBridgeError as e:
		print(e)
		rospy.loginfo(e)
		
def transBroad():
	br = tf2_ros.TransformBroadcaster()
	t = TransformStamped()
	
	t.header.stamp = rospy.Time.now()
	t.header.frame_id = "head_camera_depth_frame"
	t.child_frame_id = "target_object"
	t.transform.translation.x = 0
	t.transform.translation.z = 0
	q = tf_conversions.transformations.quaternion_from_euler(0, 0, 0)
	t.transform.rotation.x = q[0]
	t.transform.rotation.y = q[1]
	t.transform.rotation.z = q[2]
	t.transform.rotation.w = q[3]
	
	br.sendTransform(t)

def ContourSub(msg):
	global values
	global green
	
	if len(msg.moments) > 0:
		values[0] = msg.moments[0].center.x
		values[1] = msg.moments[0].center.y
		values[2] = msg.moments[0].length
		values[3] = msg.moments[0].area
		green = True
		
	else:
		green = False


def pubvel():
	global values
	global green
	global head_client
	global depth
	global reset
	
	looking_at_object = False
	
	# Initialise the ROS system and become a node
	rospy.init_node('LookAndApproach', anonymous=False)
	
	# Create a publisher object
	pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1000)
    
    # Subscribe to contour
	rospy.Subscriber('/contour_moments/moments', MomentArrayStamped, ContourSub)
	rospy.Subscriber("/head_camera/depth_registered/image_raw", Image, cb_depthImage)
	
	#rospy.spin()
	
	# Loop at 10Hz until the node is shutdown
	rate = rospy.Rate(10)
	
	rospy.loginfo("Waiting for head_controller...")
	head_client = actionlib.SimpleActionClient("head_controller/follow_joint_trajectory", FollowJointTrajectoryAction)
	head_client.wait_for_server()
	rospy.loginfo("...connected.")

	head_pos(0, 0)
	while head_moving:
		rate.sleep()
	
	while not rospy.is_shutdown():
		msg = Twist()
		
		if green:
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
					
					elif values[1] > 380 or values[1] < 100:
						rospy.loginfo("Object not in center. Correcting")
						msg.linear.x = 0
						pub.publish(msg)
						head_pos_center()
						while head_moving:
							rate.sleep()
						
					else:
						rospy.loginfo("Target in center of frame, approaching")
						rospy.loginfo("Approaching")
						msg.linear.x = 0.3
						pub.publish(msg)

			else:
				msg.linear.x = 0
				pub.publish(msg)
				if values[1] > 380 or values[1] < 100:
					head_pos_center()
				rospy.loginfo("Reached object. Depth = %.2f",depth)


					
		'''				
		else:
			while not green:
				msg.angular.z = 1
				pub.publish(msg)
				rate.sleep()'''
		
		# Wait until it's time for another iteration
		rate.sleep()


if __name__ == '__main__':
	try:
		pubvel()
	except rospy.ROSInterruptException:
		pass
