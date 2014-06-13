cmake_minimum_required(VERSION 2.8.3)
project(face_detector)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs 
  geometry_msgs 
  std_srvs
  cv_bridge
  stereo_msgs
  sensor_msgs
  tf
  rosbag
  image_transport
  people_msgs
  image_geometry
  actionlib_msgs
  actionlib
  rostest
)

#Look for OpenCV
find_package(OpenCV REQUIRED)

#Look for Boost
find_package(Boost REQUIRED COMPONENTS signals system thread)

## Generate actions in the 'action' folder
add_action_files(
   DIRECTORY    action
   FILES   FaceDetector.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES 
   actionlib_msgs
   people_msgs   
)

## Specify additional locations of header files
include_directories(
  include ${catkin_INCLUDE_DIRS}
)

## Declare a cpp executable
add_executable(face_detector
               src/face_detection.cpp 
               src/faces.cpp
)

## Add cmake target dependencies of the executable/library
add_dependencies(face_detector 
                 face_detector_generate_messages_cpp people_msgs_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(face_detector ${catkin_LIBRARIES} ${OpenCV_LIBS} ${Boost_LIBRARIES})

#############
## Testing ##
#############

## Download test data (call "$make tests" in catkin build dir)
catkin_download_test_data(
face_detector_noface_test_diamondback.bag
http://pr.willowgarage.com/data/face_detector/face_detector_noface_test_diamondback.bag
DESTINATION ${PROJECT_SOURCE_DIR}/test
MD5 37f043be780a4511c853379defdd9855
)

## Download test data
catkin_download_test_data(
face_detector_withface_test_diamondback.bag
http://pr.willowgarage.com/data/face_detector/face_detector_withface_test_diamondback.bag
DESTINATION ${PROJECT_SOURCE_DIR}/test
MD5 59126117e049e69d577b7ee27251a6f8)

## Add rostests (call "$make test" in package dir in catkin build space)
add_rostest(test/face_detector_true_rtest.xml)
add_rostest(test/face_detector_false_rtest.xml)
