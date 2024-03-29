#! /usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

from math import pi

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface', anonymous=True)


## Instantiate a `RobotCommander`_ object. This object is the outer-level interface to
## the robot:
robot = moveit_commander.RobotCommander()
## Instantiate a `PlanningSceneInterface`_ object.  This object is an interface
## to the world surrounding the robot:
scene = moveit_commander.PlanningSceneInterface()  

panda_arm = moveit_commander.MoveGroupCommander("panda_arm")
hand = moveit_commander.MoveGroupCommander("hand")
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)
panda_arm.set_pose_reference_frame('panda_link0')
end_effector_link = panda_arm.get_end_effector_link()
#[-0.7068246908767175, 4.987609811687592e-07, -0.7073887590049329, 4.946056750221776e-07]
pose_taget = geometry_msgs.msg.Pose()
pose_taget.position.x = 0.4
pose_taget.position.y = 0.2
pose_taget.position.z = 0.3
pose_taget.orientation.x = 1
pose_taget.orientation.y = 0
pose_taget.orientation.z = 0
pose_taget.orientation.w = 0

panda_arm.set_start_state_to_current_state()
panda_arm.set_pose_target(pose_taget,end_effector_link)

traj = panda_arm.plan()

panda_arm.execute(traj)
#ur5_manipulator.set_named_target(pose_taget)
#ur5_manipulator.go()

#rospy.sleep(2)

#ur5_gripper.set_named_target("grasp")
#ur5_gripper.go()

rospy.sleep(2)

##panda_arm.set_named_target("ready")
##panda_arm.go()

print "Current Pose:"
print panda_arm.get_current_pose()

##print "Reference frame: %s" % group.get_planning_frame()
##print "End effector: %s" % group.get_end_effector_link()
##print "Robot Groups:"
##print robot.get_group_names()
##print "Current Joint Values:"
##print group.get_current_joint_values()
##print "Current Pose:"
##print group.get_current_pose()
##print "Robot State:"
##print robot.get_current_state()
rospy.sleep(4)

moveit_commander.roscpp_shutdown()
