cmake_minimum_required(VERSION 2.8.3)
project(people_msgs)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
)

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
   DEPENDENCIES   geometry_msgs
)

## export the message runtime dependency
catkin_package(
  CATKIN_DEPENDS   message_runtime
)
