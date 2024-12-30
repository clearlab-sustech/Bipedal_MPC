cmake_minimum_required(VERSION 3.5)
project(mujocodl)

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

set(TARGET_DEPENDENCE
  ament_cmake
)

# find dependencies
find_package(ament_cmake REQUIRED)

add_library(mujoco SHARED IMPORTED)
set_target_properties(mujoco PROPERTIES IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/mujoco/lib/libmujoco.so)

file(GLOB_RECURSE source ${CMAKE_CURRENT_SOURCE_DIR}/mujoco/simulate/*.cc ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cc)

add_library(${PROJECT_NAME} SHARED ${source})

target_link_libraries(${PROJECT_NAME}
  mujoco glfw pthread rt ${CMAKE_DL_LIBS}
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/mujoco/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(${PROJECT_NAME} ${TARGET_DEPENDENCE})
ament_export_targets(${PROJECT_NAME}_Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(${TARGET_DEPENDENCE})

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY mujoco/include/
  DESTINATION include
)

file(GLOB_RECURSE sim_headers ${CMAKE_CURRENT_SOURCE_DIR}/mujoco/simulate/*.h)

install(
  FILES ${sim_headers}
  DESTINATION include/simulate
)

file(GLOB_RECURSE libs ${CMAKE_CURRENT_SOURCE_DIR}/mujoco/lib/*)

install(FILES ${libs}
  DESTINATION lib)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}_Targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_package()
