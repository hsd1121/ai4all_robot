#!/usr/bin/env python
import rospy
import actionlib
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray
import random

import hector_moveit_navigation.msg
import octomap_msgs.msg

frontiers = PoseArray()#array of poses (message with position, xyz and orientation)

# Get frontiers
def callback(data):
	global frontiers
	frontiers = data

def navigator_client():
	# Initialize ROS node
	rospy.init_node('navigator_client_py')
	rospy.loginfo("Initialized node")#communication with robot

	# Initialize frontier subscriber
	rospy.Subscriber("frontiers", PoseArray, callback)

	# Initialize Action Client for Navigation- telling robot where to move
	client = actionlib.SimpleActionClient('hector_navigator', hector_moveit_navigation.msg.NavigationAction)

	# Wait to connect to server
	rospy.loginfo("Waiting for server")
	client.wait_for_server()

	# Initialize navigation client goal message- where to move robot
	goal = hector_moveit_navigation.msg.NavigationGoal()
	rate = rospy.Rate(2)

	if not rospy.is_shutdown():
		goal.goal_pose.position.x = 0;
		goal.goal_pose.position.y = 0;
		goal.goal_pose.position.z = 1;
		goal.goal_pose.orientation.x = 0;
		goal.goal_pose.orientation.y = 0;
		goal.goal_pose.orientation.z = 0;
		goal.goal_pose.orientation.w = 1;
		


		rospy.loginfo("Sending takeoff position (0, 0, 1)")
		client.send_goal(goal)

		rospy.loginfo("Waiting for takeoff")
		client.wait_for_result()
	# Keep running while ros is okay

	while not rospy.is_shutdown():
		global frontiers

		# Get current frontiers
		current_frontiers = frontiers
		random_frontiers_list = []
		rospy.loginfo("Selecting 5 random poses")
		for i in range (0, 5):
			number = random.randint(0, len(current_frontiers.poses)-1)
			while number in random_frontiers_list:
				number = random.randint(0, len(current_frontiers.poses)-1)
			random_frontiers_list.append(number)

		''' 
		Optimize the for loop below to more efficiently navigate to frontiers
		'''
		rospy.loginfo("Converting random pose list to pose array")
		random_frontiers = PoseArray()
		for item in random_frontiers_list:
			rospy.loginfo("Random number: %d", item)
			temp_pose = current_frontiers.poses[item]
			random_frontiers.poses.append(temp_pose)

		rospy.loginfo("Random Frontier Pose Array Size: %d", len(random_frontiers.poses))
		for pose in random_frontiers.poses:
			goal.goal_pose = pose
			rospy.loginfo("Sending goal position (%f, %f, %f)", pose.position.x, pose.position.y, pose.position.z)
			rospy.loginfo("With pose (%f, %f, %f, %f)", pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
			client.send_goal(goal)

			rospy.loginfo("Waiting for result")
			client.wait_for_result()

		# Sleep for half a second
		rate.sleep()
	

	

if __name__ == '__main__':
	try:
		navigator_client()
	except rospy.ROSInterruptException:
		pass
