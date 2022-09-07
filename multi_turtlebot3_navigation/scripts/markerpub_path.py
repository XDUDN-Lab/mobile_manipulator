#!/usr/bin/env python
import rospy, sys
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from math import fabs

def markerPub():
	
	# Create a marker publisher.
	marker_puber = rospy.Publisher('robot1_path', MarkerArray, queue_size=10)
	rospy.init_node('markerPub', anonymous=True)
	rate = rospy.Rate(20)
	
	# Get the trails of the end effector from moveit API.
	
	# Init the Marker and clear the trails created before.
	marker = Marker()
	marker.points = []
	marker.ns = "robot1_path"
	marker.header.frame_id = 'robot_1/odom'
	marker.id = 0
	marker.type = 4
	marker.action = Marker.ADD
	marker.scale.x = 0.02
	marker.scale.y = 0.02
	marker.scale.z = 0
	marker.color.a = 1.0
	marker.color.r = 0
	marker.color.g = 0
	marker.color.b = 1
	
        goal = Marker()
        goal.ns = "robot1_path"
	goal.header.frame_id = 'robot_1/map'
	goal.id = 1
	goal.type = 1
	goal.action = Marker.ADD
	goal.scale.x = 0.3
	goal.scale.y = 0.3
	goal.scale.z = 0.0001
	goal.color.a = 1.0
	goal.color.r = 1
	goal.color.g = 1
	goal.color.b = 0
	goal.pose.position.x = 3.0
        goal.pose.position.y = 3.0

        marker_array = MarkerArray()
        marker_array.markers.append(marker)
        marker_array.markers.append(goal)
        marker_puber.publish(marker_array)

	# Publish trails contantly.
	while not rospy.is_shutdown():
		
		# marker_array[0].header.stamp = rospy.Time().now()
		# marker_array[1].header.stamp = rospy.Time().now()
		
		# points and line type use marker.point and arrow et. use pose
                odom = rospy.wait_for_message('robot_1/odom', Odometry)

                position = odom.pose.pose.position
                # reset model
                if (fabs(position.x) < 0.01 and fabs(position.y) < 0.01):
                    marker_array.markers[0].points = []
                else:
		    marker_array.markers[0].points.append(position)
		marker_puber.publish(marker_array)
		rate.sleep()
	

if __name__ == "__main__":
	try:
		markerPub()
	except rospy.ROSInterruptException:	
            rospy.signal_shutdown("shutdown..")

