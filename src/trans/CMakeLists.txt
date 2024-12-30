cmake_minimum_required(VERSION 3.5)
project(trans)

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
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(TARGET_DEPENDENCE
  ament_cmake
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  rosidl_default_generators
)

rosidl_generate_interfaces(${PROJECT_NAME}
  msgs/ActuatorCmds.msg
  msgs/TouchSensor.msg
  srv/SimulationReset.srv
  srv/GaitSwitch.srv
  DEPENDENCIES ament_cmake std_msgs geometry_msgs sensor_msgs nav_msgs
)

ament_package()
