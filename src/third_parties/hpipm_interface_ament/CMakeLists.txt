cmake_minimum_required(VERSION 3.8)
project(hpipm_interface_ament)

find_package(ament_cmake REQUIRED)
find_package(blasfeo_ament REQUIRED)
find_package(hpipm_ament REQUIRED)

include(FetchContent)

# Define directories
set(HPIPM_DEVEL_PREFIX ${CMAKE_INSTALL_PREFIX} CACHE STRING "HPIPM_Interface install path")
set(HPIPM_INCLUDE_DIR ${HPIPM_DEVEL_PREFIX}/include)
set(HPIPM_LIB_DIR ${HPIPM_DEVEL_PREFIX}/lib)
set(HPIPM_DOWNLOAD_DIR ${CMAKE_CURRENT_BINARY_DIR}/download)
set(HPIPM_BUILD_DIR ${CMAKE_CURRENT_BINARY_DIR}/build)

# Create directories if they do not exist
file(MAKE_DIRECTORY ${HPIPM_INCLUDE_DIR})
file(MAKE_DIRECTORY ${HPIPM_LIB_DIR})
file(MAKE_DIRECTORY ${HPIPM_DOWNLOAD_DIR})
file(MAKE_DIRECTORY ${HPIPM_BUILD_DIR})

# Download & build source
FetchContent_Declare(hpipmDownload
  GIT_REPOSITORY https://github.com/mayataka/hpipm-cpp.git
  GIT_TAG main
  UPDATE_COMMAND ""
  SOURCE_DIR ${HPIPM_DOWNLOAD_DIR}
  BINARY_DIR ${HPIPM_BUILD_DIR}
  BUILD_COMMAND "$(MAKE) shared_library"
  INSTALL_COMMAND "$(MAKE) install_shared"
)
FetchContent_MakeAvailable(hpipmDownload)

# Copy header to where catkin expects them
file(GLOB_RECURSE HEADERS "${HPIPM_DOWNLOAD_DIR}/include/*.h")

foreach(HEADER_FILE ${HEADERS})
  message(STATUS "FOUND HEADER: " ${HEADER_FILE})
  file(COPY ${HEADER_FILE} DESTINATION ${HPIPM_INCLUDE_DIR})
endforeach()

ament_export_libraries(hpipm-cpp)
ament_package()