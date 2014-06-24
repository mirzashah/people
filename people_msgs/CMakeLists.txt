# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(people_msgs)
# Load catkin and all dependencies required for this package
# TODO: remove all from COMPONENTS that are not catkin packages.
find_package(catkin REQUIRED COMPONENTS message_generation geometry_msgs)

# include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})
# CATKIN_MIGRATION: removed during catkin migration
# cmake_minimum_required(VERSION 2.4.6)

# CATKIN_MIGRATION: removed during catkin migration
# include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

## Generate messages in the 'msg' folder
add_message_files(
   DIRECTORY msg
   FILES
   People.msg
   Person.msg
   PersonStamped.msg
   PositionMeasurementArray.msg
   PositionMeasurement.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES   geometry_msgs std_msgs
)

catkin_package(
    CATKIN_DEPENDS message_runtime geometry_msgs std_msgs
)
