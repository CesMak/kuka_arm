# Prequisities
*  ROS Indigo or higher
* Control packages:
```
sudo apt-get install ros-kinetic-gazebo-ros
sudo apt-get install ros-kinetic-gazebo-ros-control
```

# Installation
* mkdir kuka_arm
* cd kuka_arm
* mkdir src
* cd src
* git clone git@github.com:CesMak/kuka_arm.git
* cd ..
* catkin init
* catkin build
* source devel/setup.bash
* roslaunch kuka_manipulator_description test_kr5.launch 

# Bring up in RVIZ and change joints
```
roslaunch kr5_description start_kr5_description_rviz.launch
```

![kuka_description of robot](https://github.com/CesMak/kuka_arm/blob/master/doc/kuka_arm_rviz.png)

## Moveit
* Connect Gazebo Simulation to MoveIt

```
roslaunch kuka_kr5_gazebo rviz_connected_with_gz_using_moveit.launch
```

![rviz-moveit-gazebo](https://github.com/CesMak/kuka_arm/blob/master/doc/moveit.png)

# Visualize in Gazebo
* This file is using the ros_control package and uses Position controllers
* TODO: control a continuous law with position controllers (write a node about that)

```
roslaunch kuka_kr5_gazebo gz_connected_with_rviz.launch
```

![kuka_arm gazebo](https://github.com/CesMak/kuka_arm/blob/master/doc/gz_kr5.png)


## Send a command to gazebo
(start by clicking the start simulation button in gazebo!)

```
rostopic pub /kuka/link_1_controller/command std_msgs/Float64 "data: 0.5"
```

# Sources
For more information about this KUKA KR5 Robot out of the ros_industrial package see:

https://github.com/ros-industrial/kuka_experimental/tree/indigo-devel/kuka_kr5_support

Also in terms of licenses etc. please refer to the above website.

# Problems
* Endeffector is trembling a lot in gazebo see the discussion [on ros.answers](https://answers.ros.org/question/290181/choose-right-ros_control-configuration-for-a-kuka-arm/)
* For DH-Table of this kuka_kr5_robot please check out [kuka_experimental_issue](https://github.com/ros-industrial/kuka_experimental/issues/130)

# Links to other ROS-arm-manipulators:
* https://github.com/sinaiaranda-CIDESI/motoman_mh6-10
* https://github.com/orsalmon/kuka_manipulator_gazebo
* Connect Robot to MoveIt: https://www.youtube.com/watch?v=j6bBxfD_bYs&t=638s
