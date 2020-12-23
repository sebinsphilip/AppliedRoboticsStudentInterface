cmake_minimum_required(VERSION 2.8.3)
project(student_project)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

# find_package(catkin REQUIRED COMPONENTS
#  base
#  octomap_msgs
#  pluginlib
#  roscpp
#  rospy
#  std_msgs
#  core_planning_utils
#  core_planning_state_space
#  core_map_representation_interface
#
# )

find_package(OpenCV REQUIRED )
find_package(project_interface REQUIRED )
find_package(ompl REQUIRED)

FIND_PACKAGE(Boost COMPONENTS program_options REQUIRED)
INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS})
#TARGET_LINK_LIBRARIES(target ${Boost_LIBRARIES})
# find_package(base REQUIRED)


#set (base_INCLUDE_DIRS "/opt/ros/kinetic/include/ompl/")

message("path"${Boost_INCLUDE_DIRS} "path2" ${Boost_LIBRARIES})

## Specify additional locations of header files
include_directories(
 include
 ${OpenCV_INCLUDE_DIRS}
 ${project_interface_INCLUDE_DIRS}
 ${OMPL_INCLUDE_DIRS}
 #${base_INCLUDE_DIRS}
)


# include_directories(ompl
#   ompl-1.5.1
# )

## CREATE EXECUTABLE LIBRARY
add_executable(calibrate
   src/camera_calibration.cpp
)

add_executable(demo_match_template
   src/match_template.cpp
)

add_executable(demo_ocr
   src/ocr.cpp
)

add_executable(ompl_test
   src/planning_ompl.cpp
)


## CREATE EXECUTABLE LIBRARY
add_library(student SHARED
   src/student_interface.cpp
)

# add_library(ompl
#    ompl-1.5.1/src/ompl/geometric/planners/rrt/src/RRTstar.cpp
# )

# target_include_directories (ompl
# 	PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}
# )

target_link_libraries(student
  ${OpenCV_LIBRARIES}
  ${OMPL_LIBRARIES}
)
target_link_libraries(ompl_test
  ${OMPL_LIBRARIES}
	 ${Boost_LIBRARIES}

)
target_link_libraries(demo_match_template
  ${OpenCV_LIBRARIES}
)

target_link_libraries(demo_ocr
  tesseract
  ${OpenCV_LIBRARIES}
)

target_link_libraries(calibrate
  ${OpenCV_LIBRARIES}
)
