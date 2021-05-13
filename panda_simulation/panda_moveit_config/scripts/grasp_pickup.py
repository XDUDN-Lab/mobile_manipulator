#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
from time import sleep


def pickup():
	os.system("python3 ~/catkin_ws/src/panda_ros/panda_moveit_config/scripts/gripper_open.py")
	print ("============ task1: gripper open, waiting a minute ... ")
	# sleep(0.5)

	os.system("python3 ~/catkin_ws/src/panda_ros/panda_moveit_config/scripts/arm_init.py")
	print ("============ task2: arm init, waiting a minute ... ")
	# sleep(0.5)

	os.system("python3 ~/catkin_ws/src/panda_ros/panda_moveit_config/scripts/gripper_close.py")
	print ("============ task3: gripper close, waiting a minute ... ")
	sleep(0.5)

	os.system("python3 ~/catkin_ws/src/panda_ros/panda_moveit_config/scripts/arm_pickup.py")
	print ("============ task4: arm pickup, waiting a minute ... ")
	# sleep(0.5)

	os.system("python3 ~/catkin_ws/src/panda_ros/panda_moveit_config/scripts/gripper_open.py")
	print ("============ task5: gripper open, waiting a minute ... ")
	# sleep(0.5)

	# os.system("python3 ~/catkin_ws/src/panda_ros/panda_moveit_config/scripts/pickup.py")
	os.system("python3 ~/catkin_ws/src/panda_ros/panda_moveit_config/scripts/finished.py")
	print ("============ task6: pickup, waiting a minute ... ")
	# sleep(1)

	print ("============ All tasks are finished ")


if __name__ == '__main__':
	pickup()
