cmake_minimum_required(VERSION 2.8.3)
project(student_proect)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(OpenCV REQUIRED )
find_package(project_interface REQUIRED )

## Specify additional locations of header files
include_directories(
 include 
 ${OpenCV_INCLUDE_DIRS}
 ${project_interface_INCLUDE_DIRS}
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
)

target_link_libraries(student
  ${OpenCV_LIBRARIES}  
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
