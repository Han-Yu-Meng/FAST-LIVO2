cmake_minimum_required(VERSION 3.10)
project(vikit)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Sophus REQUIRED)

file(GLOB_RECURSE VIKIT_SOURCES "src/*.cpp")

add_library(vikit STATIC ${VIKIT_SOURCES})

set_target_properties(vikit PROPERTIES POSITION_INDEPENDENT_CODE ON)

target_include_directories(vikit PUBLIC 
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../Sophus>
  ${EIGEN3_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
  ${Sophus_INCLUDE_DIRS}
)

target_link_libraries(vikit PUBLIC
  ${OpenCV_LIBRARIES}
  Sophus
)

target_link_libraries(vikit PRIVATE fins_sdk)