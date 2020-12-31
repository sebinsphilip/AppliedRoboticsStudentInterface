cmake_minimum_required(VERSION 2.8.3)
project(student_project)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

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
 src
 #${base_INCLUDE_DIRS}
)


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



## CREATE EXECUTABLE LIBRARY
add_library(student SHARED
   src/student_interface.cpp
   src/planning_ompl.cpp
)

add_library(dubins SHARED
   src/dubinsMP.cpp
)

target_link_libraries(student
  ${OpenCV_LIBRARIES}
  ${OMPL_LIBRARIES}
  ${Boost_LIBRARIES}
  tesseract
  ${CMAKE_SOURCE_DIR}/external_lib/libpolyclipping.so.22.0.0
  ${CMAKE_SOURCE_DIR}/external_lib/libdubins.so
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
