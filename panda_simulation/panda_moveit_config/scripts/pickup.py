#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy, sys
import moveit_commander
from control_msgs.msg import GripperCommand

class Pickup:
    def __init__(self):
        # 初始化move_group的API
        moveit_commander.roscpp_initialize(sys.argv)

        # 初始化ROS节点
        rospy.init_node('pickup', anonymous=True)
 
        # 初始化需要使用move group控制的机械臂中的arm group
        arm = moveit_commander.MoveGroupCommander('panda_arm')
        
        # 初始化需要使用move group控制的机械臂中的gripper group
        gripper = moveit_commander.MoveGroupCommander('hand')
        
        # 设置机械臂和夹爪的允许误差值
        arm.set_goal_joint_tolerance(0.001)
        arm.set_goal_orientation_tolerance(0.01)
        gripper.set_goal_joint_tolerance(0.001)
        
        end_effector_link = arm.get_end_effector_link()

        # 设置允许的最大速度和加速度
        arm.set_max_acceleration_scaling_factor(1.0)
        arm.set_max_velocity_scaling_factor(1.0)
        gripper.set_max_acceleration_scaling_factor(0.5)
        gripper.set_max_velocity_scaling_factor(0.5)

        # 设置每次运动规划的时间限制：1s
        arm.set_planning_time(1)
        gripper.set_planning_time(1)

        # 控制机械臂先回到初始化位置
        arm.set_named_target('ready')
        arm.go()
        rospy.sleep(0.5)

        # 设置夹爪的目标位置，并控制夹爪运动
        # gripper.set_joint_value_target([0.00, 0.00])
        # gripper.go()
        # rospy.sleep(0.5)

        joint = arm.get_current_joint_values()
        print("============ Final joint: \n" + str(joint))
        pose = arm.get_current_pose('panda_hand')
        print("============ Final pose: \n" + str(pose))

        # ---------------------  轨迹(1)生成，关节空间插值  ---------------------#
        
        # 设置机械臂的目标位置，使用七轴的位置数据进行描述（单位：弧度）
        #joint_positions = [0,-0.25670473774633784,-0.00011601918944936784,-2.6869,0,0.85952,0]
        #result = arm.set_joint_value_target(joint_positions)

		# 0.046795487653803924, -0.12380363056995058, 0.4225866511984071, -2.5430902871741408, 0.07810708968052182, 0.858050863791366, 0.4069853356040296

        #rospy.loginfo(str(result))
        #arm.go()
        #joint = arm.get_current_joint_values()
        #print("final joint=",joint)
        #pose = arm.get_current_pose('panda_hand')
        #print('pose=',pose)
        #rospy.sleep(1)


        # 关闭并退出moveit
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)


if __name__ == "__main__":
    try:
        Pickup()
    except rospy.ROSInterruptException:
        pass
