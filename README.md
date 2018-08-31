# Prequisities
*  Tested on ROS KINETIC AND INDIGO
*  Required Packages:
```
sudo apt-get install ros-kinetic-gazebo-ros-pkg
sudo apt-get install ros-kinetic-gazebo-ros-control
sudo apt-get install ros-kinetic-moveit
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

```
roslaunch kuka_kr5_gazebo gz_connected_with_rviz.launch
```

![kuka_arm gazebo](https://github.com/CesMak/kuka_arm/blob/master/doc/gzcontrol.png)


## Send a command to gazebo
```
rostopic pub /kuka/link_1_controller/command std_msgs/Float64 "data: 0.5"
```

Try also this:

```
rostopic pub /kuka/control_command geometry_msg/Point "x: 1.0 y: 0.0   z: 0.0"
```

```
rostopic pub /kuka/control_command geometry_msg/Point "x: 2.0 y: 0.0   z: 0.0"
```

# Controller

A PID Controller and a State Space Controller is implemented. To activate the PID or the state space controller please see
the kuka_kr5_control/controller_node.yaml /controller_type value.

## PID
Note that PID Control does not work, cause the Ball Joints cannot be influenced directly but indirectly by the plate.

## State Space Controller
The scripts to get the control Matrix R and the Filter Matrix F can be found in
kuka_kr5_conrol/scripts

Documentation of the Modelling and a precise description can be found at:


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
