cmake_minimum_required(VERSION 2.8.3)

project(kuka_manipulator_description)

find_package(catkin REQUIRED)

catkin_package()

#if (CATKIN_ENABLE_TESTING)
#  find_package(roslaunch REQUIRED)
#  roslaunch_add_file_check(test/roslaunch_test.xml)
#endif()

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY urdf DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY meshes DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
