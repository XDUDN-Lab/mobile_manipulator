#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, sys
import moveit_commander
from visualization_msgs.msg import Marker
from moveit_commander import MoveGroupCommander
from geometry_msgs.msg import Pose

def markerPub():
	
	# Create a marker publisher.
	marker_puber = rospy.Publisher('marker_draw', Marker, queue_size=10)
	rospy.init_node('markerPub', anonymous=True)
	rate = rospy.Rate(10)
	
	# Get the trails of the end effector from moveit API.
	moveit_commander.roscpp_initialize(sys.argv)
	arm = MoveGroupCommander('panda_arm')
	arm.set_pose_reference_frame('panda_link0')
	
	# Init the Marker and clear the trails created before.
	RobotMarker = Marker()
	RobotMarker.points = []
	RobotMarker.ns = "franka_panda_namespace"
	RobotMarker.header.frame_id = "panda_link0"
	RobotMarker.id = 0
	RobotMarker.type = 4
	RobotMarker.action = Marker.ADD
	RobotMarker.scale.x = 0.001
	RobotMarker.scale.y = 0
	RobotMarker.scale.z = 0
	RobotMarker.pose.orientation.x = 0
	RobotMarker.pose.orientation.y = 0
	RobotMarker.pose.orientation.z = 0
	RobotMarker.pose.orientation.w = 1.0
	RobotMarker.color.a = 1.0
	RobotMarker.color.r = 0.0
	RobotMarker.color.g = 0.0
	RobotMarker.color.b = 0.0
	marker_puber.publish(RobotMarker)

	# Publish trails contantly.
	while not rospy.is_shutdown():
	
		# trail
		position = arm.get_current_pose().pose.position 

		if abs(position.z - 0.373) <= 0.0001 :  # 0.4597 + 0.25
			RobotMarker.header.stamp = rospy.Time().now()
		
			# points and line type use marker.point and arrow et. use pose
			RobotMarker.points.append(arm.get_current_pose().pose.position)
		 
		marker_puber.publish(RobotMarker)
		rate.sleep()
	
	# Exit the relevant process.
	moveit_commander.roscpp_shutdown()
	moveit_commander.os._exit(0)


if __name__ == "__main__":
	try:
		markerPub()
	except rospy.ROSInterruptException:	
		moveit_commander.roscpp_shutdown()
		moveit_commander.os._exit(0)
