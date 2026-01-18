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

fins_add_node(${PROJECT_NAME}
    src/nodes.cpp
    src/LIVMapper.cpp
    src/vio.cpp 
    src/frame.cpp 
    src/visual_point.cpp
    src/voxel_map.cpp
    src/preprocess.cpp
    src/IMU_Processing.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/3rdparty/Sophus>
    ${EIGEN3_INCLUDE_DIR}
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME} PRIVATE
    OpenMP::OpenMP_CXX
    ${PCL_LIBRARIES}
    ${OpenCV_LIBRARIES}
    Sophus
    vikit
)

fins_link_ros_dependencies(${PROJECT_NAME}
    sensor_msgs
    nav_msgs
    geometry_msgs
    cv_bridge
    pcl_conversions
    livox_ros_driver2
)

target_compile_definitions(${PROJECT_NAME} PRIVATE ROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")