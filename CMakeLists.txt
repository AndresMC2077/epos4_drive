cmake_minimum_required(VERSION 3.18)
project(epos4_drive LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(eposlib REQUIRED)
find_package(ros2units REQUIRED)

# Compiles config/red_nodo5/bus.yml + epos4.eds into master.dcf
eposlib_generate_dcf(red_nodo5
  DESTINATION share/${PROJECT_NAME}/config)

set(DEFAULT_DCF ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/config/red_nodo5/master.dcf)

add_executable(epos4_torque_node src/epos4_torque_node.cpp)
target_include_directories(epos4_torque_node PRIVATE
  include
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(epos4_torque_node
  eposlib::eposlib
  eposlib::core
)
target_compile_definitions(epos4_torque_node PRIVATE
  DEFAULT_DCF="${DEFAULT_DCF}"
)
ament_target_dependencies(epos4_torque_node
  rclcpp
  std_msgs
  ros2units
)

install(TARGETS epos4_torque_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

