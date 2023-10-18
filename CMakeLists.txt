cmake_minimum_required(VERSION 3.12)
project(
        DKSave
        LANGUAGES CXX
        VERSION "2023.10.1"
)

set(CMAKE_CXX_STANDARD 20)
message(STATUS "CMAKE_CXX_PLATFORM_ID: ${CMAKE_CXX_PLATFORM_ID}")
message(STATUS "CMAKE_PREFIX_PATH: ${CMAKE_PREFIX_PATH}")
message(STATUS "CMAKE_INCLUDE_PATH: ${CMAKE_INCLUDE_PATH}")

add_compile_definitions(NOMINMAX)


add_executable(dksave src/main.cpp include/ucamera.hpp)
target_include_directories(dksave PUBLIC include)

string(REPLACE "/" "\\\\" DKSAVE_ROOT "${PROJECT_SOURCE_DIR}")
string(APPEND DKSAVE_ROOT "\\\\")
message(STATUS "DKSAVE_ROOT: ${DKSAVE_ROOT}")
target_compile_definitions(dksave PUBLIC SOURCE_ROOT="${DKSAVE_ROOT}")


include(${PROJECT_SOURCE_DIR}/cmake/instruction_extensions.cmake)
target_wih_ie(dksave avx2)


option(DKSAVE_ENABLE_K4A "with k4a cameras support" OFF)
option(DKSAVE_ENABLE_OB "with ob cameras support" OFF)
option(DKSAVE_ENABLE_PCL "with pcl point cloud library support" OFF)


# K4A
include(${PROJECT_SOURCE_DIR}/cmake/third_party/k4a.cmake)

# OB
include(${PROJECT_SOURCE_DIR}/cmake/third_party/ob.cmake)

# OpenCV
include(${PROJECT_SOURCE_DIR}/cmake/third_party/opencv.cmake)

# fmt
include(${PROJECT_SOURCE_DIR}/cmake/third_party/fmt.cmake)

# pcl
include(${PROJECT_SOURCE_DIR}/cmake/third_party/pcl.cmake)

# yaml-cpp
include(${PROJECT_SOURCE_DIR}/cmake/third_party/yaml-cpp.cmake)

# ffmpeg
include(${PROJECT_SOURCE_DIR}/cmake/third_party/ffmpeg.cmake)

# Kerbal
include(${PROJECT_SOURCE_DIR}/cmake/third_party/Kerbal.cmake)



include(GNUInstallDirs)
install(
        TARGETS dksave
)

file(GLOB dlls_lib ${PROJECT_SOURCE_DIR}/dlls/*.dll)
message(STATUS "${dlls_lib}")
foreach (e ${dlls_lib})
    install(
            FILES "${e}"
            TYPE BIN
    )
endforeach ()

include(${PROJECT_SOURCE_DIR}/cmake/cpack.cmake)
