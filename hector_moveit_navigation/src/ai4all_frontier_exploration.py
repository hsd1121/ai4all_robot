#!/usr/bin/env python
import rospy
import actionlib
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray
import math
import random

import hector_moveit_navigation.msg
import octomap_msgs.msg

from scipy.cluster.hierarchy import linkage
from scipy.cluster.hierarchy import fcluster

frontiers = PoseArray()
VisitedVoxels= []

def calculateDistance(x1,y1,z1,x2,y2,z2):
	dist = math.sqrt((x2 - x1)**2 + (y2 - y1)**2 + (z2 - z1)**2)
	return dist

def hasVisited(current_Pose):
	global VisitedVoxels
	rospy.loginfo("Length of VisitedVoxels List: %d", len(VisitedVoxels))
	for r in VisitedVoxels:
		distance = calculateDistance(r[0], r[1], r[2],current_Pose.position.x, current_Pose.position.y,current_Pose.position.z)
		if(distance < 2):
			return True
	return False

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
		global VisitedVoxels
		goal.goal_pose.position.x = 0;
		goal.goal_pose.position.y = 0;
		goal.goal_pose.position.z = 1;
		goal.goal_pose.orientation.x = 0;
		goal.goal_pose.orientation.y = 0;
		goal.goal_pose.orientation.z = 0;
		goal.goal_pose.orientation.w = 1;
		
		rospy.loginfo("Sending takeoff position (0, 0, 1)")
		client.send_goal(goal)

		VisitedVoxels.append([goal.goal_pose.position.x, goal.goal_pose.position.y, goal.goal_pose.position.z])
		rospy.loginfo("Waiting for takeoff")
		client.wait_for_result()
		rospy.loginfo("Done taking off")
	# Keep running while ros is okay

	while not rospy.is_shutdown():

		global frontiers

		# Get current frontiers
		# rospy.loginfo("Updating frontier list")
		current_frontiers = frontiers
		random_frontiers_list = []
		rospy.loginfo("Selecting 10 random poses")
		for i in range (0, 10):
			number = random.randint(0, len(current_frontiers.poses)-1)
			while number in random_frontiers_list:
				number = random.randint(0, len(current_frontiers.poses)-1)
			random_frontiers_list.append(number)
		rospy.loginfo("# of frontiers: %d", len(current_frontiers.poses))

		rospy.loginfo("Converting random pose list to pose array")
		random_frontiers = PoseArray()
		for item in random_frontiers_list:
			temp_pose = current_frontiers.poses[item]
			random_frontiers.poses.append(temp_pose)

		# "cluster" voxels by distance and eliminate voxels that are too close to others
		rospy.loginfo("Converting poses to list")
		pose_list = []
		for pose in random_frontiers.poses:
			pose_list.append([pose.position.x, pose.position.y, pose.position.z])

		rospy.loginfo("Starting linkage")
		Z = linkage(pose_list, method='complete', metric='euclidean')
		rospy.loginfo("Completed linkage")

		clusters = fcluster(Z, 2, criterion='distance')
		rospy.loginfo("# of clusters: %d", len(set(clusters)))

		cluster_frontiers = PoseArray()
		for i in range(1, len(set(clusters))+1):
			cluster_index = clusters.tolist().index(i)
			temp_pose2 = random_frontiers.poses[cluster_index]
			cluster_frontiers.poses.append(temp_pose2)

		''' 
			Optimize the for loop below to more efficiently navigate to frontiers
		'''

		for pose in cluster_frontiers.poses:
			if hasVisited(pose):
				rospy.loginfo("Goal pose of (%f, %f, %f) has been visited", pose.position.x, pose.position.y, pose.position.z)
			else:	
				global VisitedVoxels
				goal.goal_pose = pose
				client.send_goal(goal)
				rospy.loginfo("Sending goal position (%f, %f, %f)", pose.position.x, pose.position.y, pose.position.z)
				rospy.loginfo("Waiting for result")
				client.wait_for_result()
				VisitedVoxels.append( [pose.position.x, pose.position.y, pose.position.z] )

		# Sleep for half a second
		rospy.loginfo("Sleep")
		rate.sleep()


if __name__ == '__main__':
	try:
		navigator_client()
	except rospy.ROSInterruptException:
		pass
