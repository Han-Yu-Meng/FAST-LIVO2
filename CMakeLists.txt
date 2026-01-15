cmake_minimum_required(VERSION 3.16)
project(fastlivo2)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_definitions(-DBOOST_BIND_GLOBAL_PLACEHOLDERS)

add_compile_options(
    -Wno-reorder
    -Wno-unused-parameter
    -Wno-unused-variable
    -Wno-unused-but-set-variable
    -Wno-sign-compare 
    -Wno-deprecated-declarations
    -Wno-comment
    -Wno-switch
    -Wno-maybe-uninitialized 
)

set(BUILD_SOPHUS_TESTS OFF CACHE BOOL "" FORCE)
set(BUILD_SOPHUS_EXAMPLES OFF CACHE BOOL "" FORCE)
add_subdirectory(src/3rdparty/Sophus)
add_subdirectory(src/3rdparty/rpg_vikit)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Sophus REQUIRED)
find_package(OpenMP REQUIRED)

find_package(rclcpp QUIET)
find_package(sensor_msgs QUIET)
find_package(nav_msgs QUIET)
find_package(geometry_msgs QUIET)
find_package(cv_bridge QUIET)
find_package(livox_ros_driver2 QUIET)
find_package(pcl_conversions QUIET)

if(NOT MSVC)
    add_compile_options(-fexceptions -pthread)
    if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(arm|aarch64|ARM|AARCH64)")
        add_compile_options(-O3 -mcpu=native -mtune=native -ffast-math)
        add_definitions(-DARM_ARCH)
    else()
        add_compile_options(-O3 -march=native -mtune=native -funroll-loops)
        add_definitions(-DX86_ARCH)
    endif()
endif()

include(ProcessorCount)
ProcessorCount(N)
message(STATUS "Processor count: ${N}")

if(N GREATER 4)
  math(EXPR PROC_NUM "4")
  add_definitions(-DMP_EN -DMP_PROC_NUM=${PROC_NUM})
elseif(N GREATER 1)
  math(EXPR PROC_NUM "${N}")
  add_definitions(-DMP_EN -DMP_PROC_NUM=${PROC_NUM})
else()
  add_definitions(-DMP_PROC_NUM=1)
endif()

add_library(fastlivo2 SHARED
    src/nodes.cpp
    src/LIVMapper.cpp
    src/vio.cpp 
    src/frame.cpp 
    src/visual_point.cpp
    src/voxel_map.cpp
    src/preprocess.cpp
    src/IMU_Processing.cpp
)

target_include_directories(fastlivo2 PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/3rdparty/Sophus>
    ${EIGEN3_INCLUDE_DIR}
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(fastlivo2 PRIVATE fins_sdk vikit)

target_link_libraries(fastlivo2 PRIVATE
    OpenMP::OpenMP_CXX
    ${PCL_LIBRARIES}
    ${OpenCV_LIBRARIES}
    Sophus
)

if(TARGET sensor_msgs::sensor_msgs)
    target_link_libraries(fastlivo2 PRIVATE sensor_msgs::sensor_msgs)
else()
    target_include_directories(fastlivo2 PRIVATE ${sensor_msgs_INCLUDE_DIRS})
    target_link_libraries(fastlivo2 PRIVATE ${sensor_msgs_LIBRARIES})
endif()

if(TARGET nav_msgs::nav_msgs)
    target_link_libraries(fastlivo2 PRIVATE nav_msgs::nav_msgs)
else()
    target_include_directories(fastlivo2 PRIVATE ${nav_msgs_INCLUDE_DIRS})
    target_link_libraries(fastlivo2 PRIVATE ${nav_msgs_LIBRARIES})
endif()

if(TARGET geometry_msgs::geometry_msgs)
    target_link_libraries(fastlivo2 PRIVATE geometry_msgs::geometry_msgs)
else()
    target_include_directories(fastlivo2 PRIVATE ${geometry_msgs_INCLUDE_DIRS})
    target_link_libraries(fastlivo2 PRIVATE ${geometry_msgs_LIBRARIES})
endif()

if(TARGET cv_bridge::cv_bridge)
    target_link_libraries(fastlivo2 PRIVATE cv_bridge::cv_bridge)
else()
    target_include_directories(fastlivo2 PRIVATE ${cv_bridge_INCLUDE_DIRS})
    target_link_libraries(fastlivo2 PRIVATE ${cv_bridge_LIBRARIES})
endif()

if(TARGET pcl_conversions::pcl_conversions)
    target_link_libraries(fastlivo2 PRIVATE pcl_conversions::pcl_conversions)
else()
    target_include_directories(fastlivo2 PRIVATE ${pcl_conversions_INCLUDE_DIRS})
    target_link_libraries(fastlivo2 PRIVATE ${pcl_conversions_LIBRARIES})
endif()

if(TARGET livox_ros_driver2::livox_ros_driver2)
    target_link_libraries(fastlivo2 PRIVATE livox_ros_driver2::livox_ros_driver2)
else()
    target_include_directories(fastlivo2 PRIVATE ${livox_ros_driver2_INCLUDE_DIRS})
    target_link_libraries(fastlivo2 PRIVATE ${livox_ros_driver2_LIBRARIES})
endif()

target_compile_definitions(fastlivo2 PRIVATE ROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")