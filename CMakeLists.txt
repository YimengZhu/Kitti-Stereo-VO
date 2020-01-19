cmake_minimum_required(VERSION 3.5)

project(StereoVO)

set(CMAKE_CXX_COMPILER "g++")
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11 -O3")


list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

find_package(OpenCV 3.1 REQUIRED)
find_package(Sophus REQUIRED)
find_package(G2O REQUIRED)

include_directories("/usr/include/eigen3/")
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${Sophus_INCLUDE_DIRS})
include_directories(${G2O_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR}/include)

set( THIRD_PARTY_LIBS
    ${OpenCV_LIBS}
    ${Sophus_LIBRARIES}
    g2o_core g2o_stuff g2o_types_sba
)

add_subdirectory(src)
add_subdirectory(test)
