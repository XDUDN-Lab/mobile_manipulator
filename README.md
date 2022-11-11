# mobile_manipulator

## 介绍
移动机械臂作为一种新型协作机器人，受到越来越多学者的广泛关注，该功能包以bulldog移动平台与franka panda七自由度冗余机械臂所组成的移动机械臂系统为研究对象，重点设计和实现以下功能：
  1. 基于moveit!与actionlib机制的七自由度冗余机械臂运动规划算法 —— ``` manipulator_moveit_config ```
  2. 基于moveit!与Gazebo联合仿真的逆运动学求解算法，动力学验证 —— ``` manipulator_gazebo ```
  3. 基于find_object,opencv,pcl的三维物体位姿解算与自主抓取 —— ``` find-object,opencv,vision_opencv ```
  4. 基于贪婪边界搜索的自主探索算法 —— ``` bulldog_navigation ```
  5. 基于RRT-GFE的多机器人协同空间探测算法 —— ``` turtlebot3_ros ``` (已更新)
  6. 基于ar_track_alver的机械臂自主抓取实验 —— ``` panda_gazebo,gazebo2rviz```
  
目前，笔者已经在自己的笔记本上(Intel: i7-10850H, Nividia GTX2060, 32G内存)测试通过了上述所有功能包，后续该功能包将一直保持更新状态，未来也会有新的功能陆续加入，敬请期待！

## 软件依赖
[ROS melodic](http://wiki.ros.org/melodic/Installation/Ubuntu) 下所需依赖如下：
1. [find_object](https://github.com/introlab/find-object/tree/melodic-devel)
2. [kinect_v2](https://github.com/a-price/kinect_v2_description)
3. [vision_opencv](https://github.com/ros-perception/vision_opencv/tree/melodic)
4. 探索插件：[frontier_exploration](https://github.com/paulbovbel/frontier_exploration)
5. 地图融合：[m-explore](https://github.com/hasauino/m-explore)
6. [gmapping](https://github.com/ros-perception/openslam_gmapping)
7. 八叉树地图：[octomap](https://github.com/OctoMap/octomap)

## 使用说明

#### 1. MoveIt + Gazebo 联合仿真 (通过MoveIt控制Gazebo):  

```sh
roslaunch manipulator_gazebo manipulator_bringup_moveit.launch   # 存于old code文件夹下
```
or
```sh
roslaunch manipulator_gazebo mobile_manipulator_gazebo.launch
roslaunch manipulator_gazebo mobile_manipulator_moveit.launch
```

#### 2. Gazebo + Rviz (通过配置加载控制器起到控制移动机械臂的效果):  

```sh
roslaunch robots_description gazebo_manipulator_demo.launch
roslaunch bulldog_gazebo bulldog_gazebo.launch   # Only bulldog
```

#### 3. 导航+moveit:  

```sh
roslaunch manipulator_gazebo mobile_manipulator_gazebo.launch
roslaunch manipulator_gazebo mobile_manipulator_moveit.launch
roslaunch bulldog_navigation move_base_mapless_demo.launch
rosrun gmapping slam_gmapping
rosrun manipulator_moveit_config markerpub_star.py
rosrun manipulator_moveit_config moveit_star.py
```

#### 4. moveit! + marker:  

```sh
roslaunch manipulator_moveit_config demo.launch
rosrun manipulator_moveit_config markerpub_rose_curve.py
rosrun manipulator_moveit_config moveit_rose_curve.py
```  

#### 5. explore-lite 探索 + moveit + gazebo:  

```sh
roslaunch manipulator_gazebo mobile_manipulator_gazebo.launch
roslaunch manipulator_gazebo mobile_manipulator_moveit.launch   # 需更改manipulator_moveit_config功能包中的moveit_rviz.launch文件中的rviz场景
roslaunch bulldog_navigation move_base_mapless_demo.launch
rosrun gmapping slam_gmapping
roslaunch explore_lite explore.launch
```

#### 6. 多线雷达导航 + 建图:  

```sh
roslaunch bulldog_gazebo bulldog_gazebo.launch
roslaunch bulldog_navigation pointcloud_to_laserscan.launch
roslaunch bulldog_navigation move_base_mapless_demo.launch
roslaunch bulldog_navigation gmapping.launch
rosrun rqt_robot_steering rqt_robot_steering
```

#### 7. frontier 探索:  

```sh
roslaunch bulldog_gazebo bulldog_gazebo.launch
roslaunch bulldog_navigation exploration_demo.launch
```

#### 8. 自主探索 + 建图:  

```sh
roslaunch bulldog_gazebo bulldog_gazebo.launch  
roslaunch bulldog_navigation pointcloud_to_laserscan.launch  # 多线雷达需要先将三维点云/velodyne_points转换为二维/scan,单线雷达则不需要
roslaunch bulldog_navigation move_base_mapless_demo.launch
roslaunch bulldog_navigation gmapping.launch
rosrun bulldog_navigation prometheus
```

#### 9. find_object_2d franka 视觉抓取:  

```sh
roslaunch panda_gazebo panda_bringup_gazebo.launch
roslaunch panda_gazebo panda_bringup_moveit.launch
roslaunch find_object_2d find_object_3d.launch
rosrun opencv tf_listener.py
rosrun grasp_control control.py
```

#### 10. RRT-GFE 多机器人协同空间探索:  

```sh
roslaunch multi_turtlebot3_navigation exploring_two.launch
roslaunch multi_rrt_exploration rrt_two_robots.launch
```

#### 11. ar_track_alver + gazebo2rviz 机械臂自主抓取

```sh
roslaunch panda_gazebo panda_bringup_gazebo.launch
roslaunch panda_gazebo panda_bringup_moveit.launch
roslaunch gazebo2rviz gazebo2rviz.launch
roslaunch gazebo2rviz gazebo2moveit.launch
```
```sh
roslaunch panda_gazebo ar_track_kinect.launch
rosrun panda_gazebo ar_track_follower.py
```
