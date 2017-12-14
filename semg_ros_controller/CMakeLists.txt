cmake_minimum_required(VERSION 2.8.3)
project(semg_ros_controller)

execute_process(COMMAND bash -c "cd ${CMAKE_SOURCE_DIR}; npm install")

find_package(catkin REQUIRED)

catkin_package()

include_directories()
