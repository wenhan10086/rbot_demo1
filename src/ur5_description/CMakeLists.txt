cmake_minimum_required(VERSION 3.1.3)
project(ur5_description)
find_package(catkin REQUIRED)

catkin_package()

find_package(roslaunch)

install(DIRECTORY meshes urdf launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})