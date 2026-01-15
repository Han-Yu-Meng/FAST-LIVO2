cmake_minimum_required(VERSION 3.10)
project(Sophus)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(CMAKE_COMPILER_IS_GNUCXX)
   set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")
   add_definitions("-Wall -Wno-unused-variable -Wno-unused-but-set-variable -Wno-unknown-pragmas")
endif()

find_package(Eigen3 REQUIRED)

set(SOURCE_DIR "sophus")
set(CLASSES so2 se2 se3 so3 scso3 sim3)
set(SOURCES)
foreach(class ${CLASSES})
  list(APPEND SOURCES ${SOURCE_DIR}/${class}.cpp ${SOURCE_DIR}/${class}.h)
endforeach()

add_library(Sophus STATIC ${SOURCES})

target_include_directories(Sophus PUBLIC 
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${EIGEN3_INCLUDE_DIR}
)

option(BUILD_SOPHUS_TESTS "Build Sophus unit tests" OFF)
if(BUILD_SOPHUS_TESTS)
    foreach(class ${CLASSES})
        add_executable(test_${class} sophus/test_${class}.cpp)
        target_link_libraries(test_${class} Sophus)
    endforeach()
endif()