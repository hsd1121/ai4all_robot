#!/usr/bin/env python
import rospy
import actionlib
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray

import hector_moveit_navigation.msg
import octomap_msgs.msg
import math
VisitedVoxels= []
#VisitedVoxels.append( [goal.position.x, goal.position.y, goal.position.z] )


frontiers = PoseArray()


def Vox(voxel_List,voxel_pose):
	if voxel_List[0] ==voxel_pose.position.x and voxel_List[1] ==voxel_pose.position.y and voxel_List[2] ==voxel_pose.position.z:
		return True
	else:
		return False

def dista(x1, y1, z1, x2, y2, z2):  
	d = math.sqrt(math.pow(x2 - x1, 2) +
		math.pow(y2 - y1, 2) +
		math.pow(z2 - z1, 2)* 1.0) 

def hasVisited(current_Pose):
	global VisitedVoxels
	for r in VisitedVoxels:
		if(dista(r[0], r[1], r[2],current_Pose.position.x, current_Pose.position.y,current_Pose.position.z, ))>2:
			return False
		else:
			return True

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
	global VisitedVoxels

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
			prev_Visited = False

			for num in VisitedVoxels:
				if Vox(num,pose) or  hasVisited(pose):
					prev_Visited = True


			client.send_goal(goal)
			rospy.loginfo("Sending goal position (%f, %f, %f)", pose.position.x, pose.position.y, pose.position.z)
			rospy.loginfo("With pose (%f, %f, %f, %f)", pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
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
