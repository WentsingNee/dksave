cmake_minimum_required(VERSION 3.12)
project(DKSave CXX)

set(CMAKE_CXX_STANDARD 20)
message(STATUS "CMAKE_CXX_PLATFORM_ID: ${CMAKE_CXX_PLATFORM_ID}")
message(STATUS "CMAKE_PREFIX_PATH: ${CMAKE_PREFIX_PATH}")
message(STATUS "CMAKE_INCLUDE_PATH: ${CMAKE_INCLUDE_PATH}")


if (DEFINED CMAKE_INCLUDE_PATH)
    include_directories("${CMAKE_INCLUDE_PATH}")
endif ()

add_compile_definitions(NOMINMAX)


add_executable(dksave src/dksave.cpp)
target_include_directories(dksave PUBLIC include)


include(${CMAKE_SOURCE_DIR}/cmake/instruction_extensions.cmake)
target_wih_ie(dksave avx2)


# K4A
include(${CMAKE_SOURCE_DIR}/cmake/third_party/k4a.cmake)

# OpenCV
include(${CMAKE_SOURCE_DIR}/cmake/third_party/opencv.cmake)

# fmt
include(${CMAKE_SOURCE_DIR}/cmake/third_party/fmt.cmake)

## pcl
#include(${CMAKE_SOURCE_DIR}/cmake/third_party/pcl.cmake)

# yaml-cpp
include(${CMAKE_SOURCE_DIR}/cmake/third_party/yaml-cpp.cmake)
