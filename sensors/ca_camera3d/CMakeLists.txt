cmake_minimum_required(VERSION 2.8.3)
project(ca_camera3d)
## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
	depthimage_to_laserscan
	openni2_launch
)
