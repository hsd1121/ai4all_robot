#!/usr/bin/env python
import rospy
import actionlib
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray

import hector_moveit_navigation.msg
import octomap_msgs.msg
import math
VisitedVoxels= []

frontiers = PoseArray()


def dista(x1, y1, z1, x2, y2, z2):  
	d = math.sqrt(math.pow(x2 - x1, 2) +
		math.pow(y2 - y1, 2) +
		math.pow(z2 - z1, 2)* 1.0) 
	return d


def hasVisited(current_Pose):
	global VisitedVoxels
	rospy.loginfo("Length of VisitedVoxels List: %d", len(VisitedVoxels))
	for r in VisitedVoxels:
		distance = dista(r[0], r[1], r[2],current_Pose.position.x, current_Pose.position.y,current_Pose.position.z)
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
	# Keep running while ros is okay

	while not rospy.is_shutdown():
		global frontiers

		# Get current frontiers
		current_frontiers = frontiers
		# Navigate to all frontiers
		''' 
		Optimize the for loop below to more efficiently navigate to frontiers
		'''
		for pose in current_frontiers.poses:
			goal.goal_pose = pose
			if hasVisited(pose):
				rospy.loginfo("Goal pose of (%f, %f, %f) has been visited", pose.position.x, pose.position.y, pose.position.z)
			else:	
				global VisitedVoxels
				client.send_goal(goal)
				rospy.loginfo("Sending goal position (%f, %f, %f)", pose.position.x, pose.position.y, pose.position.z)
				rospy.loginfo("Waiting for result")
				client.wait_for_result()
				VisitedVoxels.append( [pose.position.x, pose.position.y, pose.position.z] )

		# Sleep for half a second
		rate.sleep()
	

	

if __name__ == '__main__':
	try:
		navigator_client()
	except rospy.ROSInterruptException:
		pass
