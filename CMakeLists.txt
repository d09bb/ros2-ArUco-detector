cmake_minimum_required(VERSION 3.8)
project(aruco_detector)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui objdetect)
find_package(std_msgs REQUIRED)

add_executable(aruco_node aruco_detector/aruco_node.cpp)

ament_target_dependencies(aruco_node
  rclcpp
  sensor_msgs
  cv_bridge
  std_msgs
)

target_include_directories(aruco_node PUBLIC
  ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(aruco_node
  ${OpenCV_LIBRARIES}
)

install(TARGETS
  aruco_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY resource
  DESTINATION share/${PROJECT_NAME}
)

install(FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/marker_position_controller.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
