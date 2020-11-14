cmake_minimum_required(VERSION 2.8.3)
project(esperienza2)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  rospy
  std_msgs
  message_generation
  sensor_msgs
  cv_bridge
  opencv_apps
  image_transport
  geometry_msgs
)

find_package(OpenCV REQUIRED)

# find_package(octomap REQUIRED)
# include_directories(${OCTOMAP_INCLUDE_DIRS})
# link_libraries(${OCTOMAP_LIBRARIES})

# add_message_files(
#   FILES
# )

# Generate services in the 'srv' folder
 add_service_files(
   FILES
   computePose.srv
 )

 generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
 )


catkin_package(
CATKIN_DEPENDS message_runtime
  LIBRARIES
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

include_directories(include ${OpenCV_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

add_executable(camera_viewer src/detector/camera_viewer.cpp)
target_link_libraries(camera_viewer ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(camera_viewer camera_viewer_generate_messages_cpp)

add_executable(detection_node src/detector/detection_node.cpp)
target_link_libraries(detection_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(detection_node detection_node_generate_messages_cpp)

add_executable(detection_example src/detector/detection_example.cpp)
target_link_libraries(detection_example ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
