#!/usr/bin/env python
import rospy
import actionlib
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray
import math

import hector_moveit_navigation.msg
import octomap_msgs.msg

from scipy.cluster.hierarchy import linkage
from scipy.cluster.hierarchy import fcluster

frontiers = PoseArray()


def calculateDistance(x1,y1,z1,x2,y2,z2):
	dist = math.sqrt((x2 - x1)**2 + (y2 - y1)**2 + (z2 - z1)**2)
	return dist


# Get frontiers
def callback(data):
	global frontiers
	frontiers = data

def navigator_client():
	# Initialize ROS node
	rospy.init_node('navigator_client_py')
	rospy.loginfo("Initialized node")

	# Initialize frontier subscriber
	rospy.Subscriber("frontiers", PoseArray, callback)

	# Initialize Action Client for Navigation
	client = actionlib.SimpleActionClient('hector_navigator', hector_moveit_navigation.msg.NavigationAction)

	# Wait to connect to server
	rospy.loginfo("Waiting for server")
	client.wait_for_server()

	# Initialize navigation client goal message
	goal = hector_moveit_navigation.msg.NavigationGoal()
	rate = rospy.Rate(2)
	visiting_frontiers = PoseArray()

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
		rospy.loginfo("Done taking off")
	# Keep running while ros is okay

	while not rospy.is_shutdown():

		global frontiers

		# Get current frontiers
		# rospy.loginfo("Updating frontier list")
		current_frontiers = frontiers

		rospy.loginfo("# of frontiers: %d", len(current_frontiers.poses))

		# "cluster" voxels by distance and eliminate voxels that are too close to others
		rospy.loginfo("Converting poses to list")
		pose_list = []
		for pose in current_frontiers.poses:
			pose_list.append([pose.position.x, pose.position.y, pose.position.z])

		rospy.loginfo("Starting linkage")
		Z = linkage(pose_list, method='complete', metric='euclidean')
		rospy.loginfo("Completed linkage")

		clusters = fcluster(Z, 2, criterion='distance')
		rospy.loginfo("# of clusters: %d", len(set(clusters)))
		'''
		for pose in current_frontiers.poses:
			x1 = pose.position.x
			y1 = pose.position.y
			z1 = pose.position.z
			for pose2 in current_frontiers.poses:
				x2 = pose2.position.x
				y2 = pose2.position.y
				z2 = pose2.position.z
				if calculateDistance(x1,y1,z1,x2,y2,z2) >= 2:
					visiting_frontiers.poses.append(pose2)


		'''

		''' 
			Optimize the for loop below to more efficiently navigate to frontiers
		'''
		'''
		rospy.loginfo("# of frontiers: %d", len(visiting_frontiers.poses))
		for pose in visiting_frontiers.poses:
			goal.goal_pose = pose
			rospy.loginfo("Sending goal position (%f, %f, %f)", pose.position.x, pose.position.y, pose.position.z)
			rospy.loginfo("With pose (%f, %f, %f, %f)", pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
			client.send_goal(goal)

			rospy.loginfo("Waiting for result")
			client.wait_for_result()
		'''
		# Sleep for half a second
		rospy.loginfo("Sleep")
		rate.sleep()


if __name__ == '__main__':
	try:
		navigator_client()
	except rospy.ROSInterruptException:
		pass