#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import roslib
import rospy
import actionlib
from std_msgs.msg import Float64
import trajectory_msgs.msg
import control_msgs.msg
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryAction, JointTrajectoryGoal, FollowJointTrajectoryAction, FollowJointTrajectoryGoal


class Joint:
	def __init__(self, motor_name):
		# arm_name = "panda"
		self.name = motor_name
		# self.jta = actionlib.SimpleActionClient('/'+self.name+'/arm_position_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
		self.jta = actionlib.SimpleActionClient('/panda/arm_position_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
		rospy.loginfo('Waiting for joint trajectory action')
		self.jta.wait_for_server()
		rospy.loginfo('Found joint trajectory action!')
	
	def move_joint(self, angles):
		goal = FollowJointTrajectoryGoal()
		# char = self.name[0] #either 'f' or 'b'
		goal.trajectory.joint_names = ['panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4', 'panda_joint5', 'panda_joint6', 'panda_joint7']
		point = JointTrajectoryPoint()
		point.positions = angles
		point.time_from_start = rospy.Duration(1)
		goal.trajectory.points.append(point)
		self.jta.send_goal_and_wait(goal)
	
def main():
			arm = Joint('panda')
			arm.move_joint([0,-0.5235,0,-2.0943,0,0,0.5])
			arm.move_joint([-1.5708,-0.5235,0,-2.0943,0,0,0])
			arm.move_joint([-1.5708,0.289515,-0.000185,-1.947392,0,0.6661,0])


if __name__ == '__main__':
		rospy.init_node('joint_position_tester')
		main()
