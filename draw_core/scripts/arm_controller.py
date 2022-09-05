#!/usr/bin/env python
import sys
import rospy
import time
import copy
import math

# moveit stuff
import moveit_commander

# msg stuff
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import String

class Arm_Contrl():
    def __init__(self):
        self.manipulator = moveit_commander.RobotCommander()
        self.group_name = "panda_arm"
        self.group = moveit_commander.MoveGroupCommander(self.group_name)

        # initial settings
        reference = String()
        reference = "world"
        self.group.set_pose_reference_frame(reference)
        self.group.allow_replanning(True)
        self.group.set_max_velocity_scaling_factor(1.0)      
        self.group.set_max_acceleration_scaling_factor(1.0)
        self.group.set_goal_orientation_tolerance(0.001)
        self.group.set_goal_position_tolerance(0.001)
        self.group.set_planning_time(1.0)

        # for drawing settings
        self.line_gap = 0.01

        # for real world settings
        self.up_pen_height = 0.45
        self.low_pen_height = 0.35

    def get_current_pos(self):
        return self.group.get_current_pose().pose

    def go_home(self):
        # goal = geometry_msgs.msg.Pose()
        # goal = self.group.get_current_pose().pose
        joint_positions = [-4.7936863616275244e-05, -0.19749796374007203, 5.808276348542736e-05, -2.1737422791510257, -8.948672891222956e-05, 0.4056027167450059, 3.044002774592631e-05]
        self.group.set_joint_value_target(joint_positions)
        # goal.position.x = -0.2
        # goal.position.y = -0.2
        # goal.position.z = 1.05
        # goal.orientation.x = 0.0
        # goal.orientation.y = 0.0
        # goal.orientation.z = 0.0
        # goal.orientation.w = 1.0

        # self.group.set_pose_target(goal)
        plan = self.group.go(wait = True)
        time.sleep(1.0)
        self.group.clear_pose_targets()
        print("I am ready now")

    def move(self, goal_state):
        # get current state
        current_state = geometry_msgs.msg.Pose()
        current_state = self.group.get_current_pose().pose
        self.group.set_pose_target(goal_state)

        # plan and execute
        plan = self.group.go(wait = True)
        time.sleep(1.0)
        # self.group.stop()
        self.group.clear_pose_targets()
        print("achieve move goal")

    def draw_line(self, end_point):
        # get current state
        current_state = geometry_msgs.msg.Pose()
        current_state = self.group.get_current_pose().pose

        line_points = []
        line_points.append(current_state)
        # delta_x = end_point.position.x - current_state.position.x
        # delta_y = end_point.position.y - current_state.position.y
        # dist = math.sqrt(delta_x * delta_x + delta_y * delta_y)
        # # cos_theta = (dist * dist + delta_x * delta_x - delta_y * delta_y) / 2 * dist * delta_x
        # # sin_theta = math.sqrt(1 - cos_theta * cos_theta)
        # num_points = int(dist / self.line_gap)
        # print("dist", dist)
        # print("num_points", num_points)

        temp_point = geometry_msgs.msg.Pose()
        temp_point = copy.copy(current_state) 

        # for i in range(num_points):
        #     temp_point.position.x += delta_x / num_points
        #     temp_point.position.y += delta_y / num_points
        #     line_points.append(temp_point)
        line_points.append(end_point)
        
        # draw the line
        (line_traj, fraction) = self.group.compute_cartesian_path(line_points, self.line_gap, 0,avoid_collisions= False)
        self.group.execute(line_traj, wait=True)
        time.sleep(0.25)
                
    def up_pen(self):   #Up the pen
        current_state_ = geometry_msgs.msg.Pose()
        current_state_ = self.group.get_current_pose().pose
        current_state_.position.z = self.up_pen_height
        self.draw_line(current_state_)

    def down_pen(self):   #Down the pen
        current_state_ = geometry_msgs.msg.Pose()
        current_state_ = self.group.get_current_pose().pose
        current_state_.position.z = self.down_pen
        self.draw_line(current_state_)


# get current pose
if __name__=="__main__":
    moveit_commander.roscpp_initialize(sys.argv)     
    rospy.init_node('drawing_control', anonymous = True)
    
    # test plan  
    manipulator = moveit_commander.RobotCommander()
    group_name = "panda_arm"
    group = moveit_commander.MoveGroupCommander(group_name)
    rate = rospy.Rate(0.5) # 0.5sec
    while not rospy.is_shutdown():
        p = group.get_current_pose().pose
        j = group.get_current_joint_values()
        print("current pose", p)
        print("current_joint_values", j)
        rate.sleep()




