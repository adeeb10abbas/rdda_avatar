cmake_minimum_required(VERSION 3.5)
project(rdda_interface)

# Compile as C++11
# add_compile_options(-std=c++11)
set(CMAKE_CXX_STANDARD 14)

# Find ament_cmake and other ROS2 components
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(Boost COMPONENTS system thread program_options REQUIRED)
find_package(rdda_interfaces_types REQUIRED)
find_package(Threads REQUIRED)
find_package(rosidl_default_runtime REQUIRED)


# Include directories
include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${trajectory_msgs_INCLUDE_DIRS}
  ${std_srvs_INCLUDE_DIRS}
  ${rdda_interfaces_types_INCLUDE_DIRS}
  ${rosidl_default_runtime_INCLUDE_DIRS}
)

# Declare a C++ library
add_library(${PROJECT_NAME}_src src/rdda_interface/shm.c)
ament_target_dependencies(${PROJECT_NAME}_src rclcpp std_msgs std_srvs)
target_link_libraries(${PROJECT_NAME}_src ${Boost_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT} rt)

# Declare a C++ executable
add_executable(${PROJECT_NAME}_node src/rdda_interface/rdda_interface.cpp)
ament_target_dependencies(${PROJECT_NAME}_node rclcpp std_msgs std_srvs sensor_msgs rdda_interfaces_types rosidl_default_runtime)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME}_src ${Boost_LIBRARIES})

# Install targets
install(TARGETS
  ${PROJECT_NAME}_node
  ${PROJECT_NAME}_src
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
	DESTINATION share/${PROJECT_NAME})

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config/
)

# Export dependencies
ament_export_dependencies(rclcpp std_msgs sensor_msgs trajectory_msgs Boost)
ament_export_dependencies(rosidl_default_runtime)

ament_package()
