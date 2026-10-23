cmake_minimum_required(VERSION 3.5)
project(lab_5)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs

  image_transport
  cv_bridge
)

find_package(OpenCV 4.4 REQUIRED)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES feature_trackers
 CATKIN_DEPENDS roscpp rospy sensor_msgs image_transport cv_bridge
 DEPENDS OpenCV
)


include_directories(${OpenCV_INCLUDE_DIRS}
  include
  ${catkin_INCLUDE_DIRS}
)

# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#  DELIVERABLE 6 (continued) | Comparing Feature Matching on Real Data
# ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~  ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
#
# Don't forget to modify the CMakeLists.txt to add your ORB and FAST+BRIEF
# feature trackers!
#
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
add_library(feature_trackers src/feature_tracker.cpp src/sift_feature_tracker.cpp src/lk_feature_tracker.cpp)

target_link_libraries(feature_trackers ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(track_features src/course_entry.cpp)
target_link_libraries(track_features feature_trackers ${catkin_LIBRARIES})


