#!/usr/bin/env python
import rospy
import actionlib
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray

import hector_moveit_navigation.msg
import octomap_msgs.msg

frontiers = PoseArray()

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
			rospy.loginfo("Sending goal position (%f, %f, %f)", pose.position.x, pose.position.y, pose.position.z)
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