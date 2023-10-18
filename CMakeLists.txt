cmake_minimum_required(VERSION 3.16.3)
project(camera_trigger_qt)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Qt5Core REQUIRED)
find_package(Qt5Quick REQUIRED)
find_package(Qt5Widgets REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(px4_msgs REQUIRED)


include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)



set(dependencies
    ament_index_cpp
    rclcpp
    sensor_msgs
    cv_bridge
    px4_msgs
)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

###########
## Build ##
###########

add_executable(${PROJECT_NAME}
  src/main.cpp
  src/main_gui.cpp
  src/ros2node.cpp
  include/${PROJECT_NAME}/ros2node.hpp
  include/${PROJECT_NAME}/main_gui.hpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  PUBLIC $<INSTALL_INTERFACE:include>)

# ament_target_dependencies(${PROJECT_NAME} SYSTEM rviz_visual_tools)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

target_link_libraries(${PROJECT_NAME}
  ${moveit_ros_planning_LIBRARIES}
  Qt5::Core
  Qt5::Quick
  Qt5::Widgets
)

target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra)

#############
## Install ##
#############

# Install executables
install(
  TARGETS
    ${PROJECT_NAME}
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install include, launch, config directories
install(
  DIRECTORY
    launch
  DESTINATION share/${PROJECT_NAME}
)

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   add_subdirectory(test)

#   # ROS2 linters, but disable copyright test. PickNik's copyright's may not conform
#   # to this test
#   # set(ament_cmake_copyright_FOUND TRUE)
#   # ament_lint_auto_find_test_dependencies()
# endif()

ament_package()
