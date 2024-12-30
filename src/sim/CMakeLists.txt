cmake_minimum_required(VERSION 3.5)
project(sim)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(trans REQUIRED)
find_package(mujocodl REQUIRED)
find_package(assets REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(yaml-cpp REQUIRED)

set(TARGET_DEPENDENCE
  ament_cmake
  ament_index_cpp
  trans
  mujocodl
  sensor_msgs
  rclcpp
  geometry_msgs
  tf2
  tf2_ros
  nav_msgs
  cv_bridge
  yaml-cpp
)

add_library(mujoco SHARED IMPORTED)
set_target_properties(mujoco PROPERTIES IMPORTED_LOCATION ${CMAKE_INSTALL_PREFIX}/lib/libmujoco.so)

add_executable(sim src/sim.cc src/sim/SimPublisher.cc)
target_include_directories(sim PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_link_libraries(sim
  ${YAML_CPP_LIBRARIES}
  mujoco
)
ament_target_dependencies(sim ${TARGET_DEPENDENCE})

install(TARGETS sim
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
