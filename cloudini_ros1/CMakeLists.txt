cmake_minimum_required(VERSION 3.16)
project(cloudini_ros1)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
)

# No LIBRARIES export: cloudini_lib is vendored as a static archive, not installed.
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp sensor_msgs std_msgs
)

##############################################################################
# Vendored Cloudini core
#
# cloudini_lib emits no plain-CMake cloudini_libConfig.cmake (only
# ament_export_targets), so find_package() is impossible from catkin.
# add_subdirectory is the only route.
##############################################################################

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

set(CLOUDINI_FORCE_VENDORED_DEPS OFF CACHE BOOL "" FORCE)  # use system libzstd/liblz4
set(CLOUDINI_BUILD_TOOLS         OFF CACHE BOOL "" FORCE)  # also skips find_or_download_mcap()
set(CLOUDINI_BUILD_BENCHMARKS    OFF CACHE BOOL "" FORCE)

# cloudini_lib gates its own gtest binary on BUILD_TESTING (its CMakeLists.txt calls
# include(CTest) then add_subdirectory(test)). Our tests are gated on
# CATKIN_ENABLE_TESTING instead, so turning this off does not disable them.
if(DEFINED CACHE{BUILD_TESTING})
  set(_cldn_prev_build_testing "$CACHE{BUILD_TESTING}")
endif()
set(BUILD_TESTING OFF CACHE BOOL "" FORCE)  # option() never overrides an existing cache entry
set(BUILD_TESTING OFF)                      # normal variable also wins under CMP0077

set(BUILD_SHARED_LIBS OFF)                  # catkin forces this ON; we want libcloudini_lib.a
set(CMAKE_POSITION_INDEPENDENT_CODE ON)     # keeps the archive usable from a future nodelet .so
set(CMAKE_DISABLE_FIND_PACKAGE_PCL TRUE)    # no bridge code path needs pcl_conversion.cpp

add_subdirectory(
  ${CMAKE_CURRENT_SOURCE_DIR}/../cloudini_lib
  ${CMAKE_CURRENT_BINARY_DIR}/cloudini_lib)

unset(CMAKE_DISABLE_FIND_PACKAGE_PCL)
unset(BUILD_TESTING)
if(DEFINED _cldn_prev_build_testing)
  set(BUILD_TESTING "${_cldn_prev_build_testing}" CACHE BOOL "" FORCE)
else()
  unset(BUILD_TESTING CACHE)
endif()

# catkin_package() pointed ARCHIVE_OUTPUT_DIRECTORY at devel/lib; keep the archive local.
set_target_properties(cloudini_lib PROPERTIES
  ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/cloudini_lib)

##############################################################################
# Bridge library
##############################################################################

add_library(cloudini_ros1 STATIC
  src/wire_format.cpp
  src/validation.cpp
  src/codec_bridge.cpp
)

target_include_directories(cloudini_ros1
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    ${catkin_INCLUDE_DIRS}
)
target_compile_features(cloudini_ros1 PUBLIC cxx_std_20)
target_compile_options(cloudini_ros1 PRIVATE -Wall -Wextra)
target_link_libraries(cloudini_ros1 PUBLIC cloudini_lib ${catkin_LIBRARIES})

# Internal archive, not exported via catkin_package(LIBRARIES): keep it out of devel/lib.
set_target_properties(cloudini_ros1 PROPERTIES
  ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})

##############################################################################
# Nodes
##############################################################################

add_executable(cloudini_compress_node   src/compress_node.cpp)
add_executable(cloudini_decompress_node src/decompress_node.cpp)
target_link_libraries(cloudini_compress_node   cloudini_ros1)
target_link_libraries(cloudini_decompress_node cloudini_ros1)

install(TARGETS cloudini_compress_node cloudini_decompress_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

catkin_install_python(PROGRAMS
  scripts/mock_cloud_publisher.py
  scripts/bridge_benchmark.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

##############################################################################
# Tests
##############################################################################

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_cloudini_ros1_wire       test/test_wire_format.cpp)
  catkin_add_gtest(test_cloudini_ros1_validation test/test_validation.cpp)
  catkin_add_gtest(test_cloudini_ros1_codec      test/test_codec_bridge.cpp)

  foreach(_t test_cloudini_ros1_wire test_cloudini_ros1_validation test_cloudini_ros1_codec)
    target_link_libraries(${_t} cloudini_ros1)
  endforeach()

  find_package(rostest REQUIRED)
  add_rostest_gtest(test_cloudini_ros1_roundtrip
    test/cloudini_roundtrip.test
    test/test_roundtrip_node.cpp
  )
  target_link_libraries(test_cloudini_ros1_roundtrip cloudini_ros1)
endif()
