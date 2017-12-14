cmake_minimum_required(VERSION 2.8.3)
project(alexa_ros_controller)

find_package(catkin REQUIRED)

catkin_package()

include_directories()

execute_process(
        COMMAND bash -c "npm install"
)