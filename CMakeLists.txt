cmake_minimum_required(VERSION 3.0 FATAL_ERROR)

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF(NOT CMAKE_BUILD_TYPE)

set(PROJECT_P2O p2o)
project(${PROJECT_P2O})

set(PROJECT_REARRANGE rearrange_pointcloud)
project(${PROJECT_REARRANGE})

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

find_package(PCL 1.14)
find_package(Eigen3 REQUIRED)

include_directories(include
        ${EIGEN3_INCLUDE_DIR}
)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_library(p2o SHARED src/p2o.cpp include/p2o.h)
add_executable(run_p2o src/run_p2o.cpp)
target_link_libraries(run_p2o p2o Eigen3::Eigen)

add_executable(${PROJECT_REARRANGE} src/rearrange_pointcloud.cpp)
target_link_libraries(rearrange_pointcloud ${PCL_LIBRARIES} Eigen3::Eigen)


set(BUILD_VIEWER OFF CACHE BOOL "Build p2o viewer")
if (PCL_FOUND)
    set(BUILD_VIEWER ON)
endif(PCL_FOUND)

if(BUILD_VIEWER)
    add_subdirectory(p2o_viewer)
endif(BUILD_VIEWER)

