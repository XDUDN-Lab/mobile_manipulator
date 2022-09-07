#!/bin/bash
 
# rosrun map_server map_saver map:=/robot_3/map -f ~/project_ros/src/turtlebot3_ros/multi_turtlebot3_navigation/map
rosrun map_server map_saver -f map_merge1 map:=/robot_1/map
