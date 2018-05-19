# Installation
mkdir kuka_arm
cd kuka_arm
mkdir src
cd src
git clone .....
cd ..
catkin init
catkin build
source devel/setup.bash

# Bring up in RVIZ and change joints
```
roslaunch kuka_kr5_description test_kr5_arc.launch 
```



# Visualize in Gazebo
```
roslaunch kuka_manipulator_gazebo manipulator_empty_world.launch 
```

# Sources
For more information about this KUKA KR5 Robot out of the ros_industrial package see:

https://github.com/ros-industrial/kuka_experimental/tree/indigo-devel/kuka_kr5_support

Also in terms of licenses etc. please refer to the above website.
