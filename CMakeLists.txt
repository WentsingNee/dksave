cmake_minimum_required(VERSION 3.12)

option(DKSAVE_ENABLE_K4A "enable k4a cameras support" OFF)
option(DKSAVE_ENABLE_OB "enable ob cameras support" OFF)
option(DKSAVE_ENABLE_PCL "enable pcl point cloud library support" OFF)

if (DKSAVE_ENABLE_K4A)
    list(APPEND VCPKG_MANIFEST_FEATURES "k4a")
endif ()

if (DKSAVE_ENABLE_OB)
    list(APPEND VCPKG_MANIFEST_FEATURES "ob")
endif ()

if (DKSAVE_ENABLE_PCL)
    list(APPEND VCPKG_MANIFEST_FEATURES "pcl")
endif ()

project(
        DKSave
        LANGUAGES CXX
        VERSION "2024.2.1"
)

set(CMAKE_CXX_STANDARD 20)
message(STATUS "CMAKE_CXX_PLATFORM_ID: ${CMAKE_CXX_PLATFORM_ID}")
message(STATUS "CMAKE_PREFIX_PATH: ${CMAKE_PREFIX_PATH}")
message(STATUS "CMAKE_INCLUDE_PATH: ${CMAKE_INCLUDE_PATH}")

add_compile_definitions(NOMINMAX)
if (CMAKE_CXX_COMPILER MATCHES "GNU")
    add_compile_options(-fconcepts)
endif ()


add_executable(dksave src/main.cpp)
target_include_directories(dksave PUBLIC include)

set(DKSAVE_ROOT "${PROJECT_SOURCE_DIR}/")
if (CMAKE_CXX_PLATFORM_ID MATCHES "Windows")
    string(REPLACE "/" "\\\\" DKSAVE_ROOT "${DKSAVE_ROOT}")
endif ()
message(STATUS "DKSAVE_ROOT: ${DKSAVE_ROOT}")
target_compile_definitions(dksave PUBLIC SOURCE_ROOT="${DKSAVE_ROOT}")


# ctre
include(${PROJECT_SOURCE_DIR}/cmake/third_party/ctre.cmake)

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


if (DKSAVE_ENABLE_OB)
    file(GLOB dlls_lib_ob ${PROJECT_SOURCE_DIR}/dlls/ob/*.dll)
endif ()


foreach (dll_lib IN LISTS dlls_lib_ob)
    message(STATUS "dll_lib: ${dll_lib}")
    install(
            FILES "${dll_lib}"
            TYPE BIN
    )
endforeach ()

include(${PROJECT_SOURCE_DIR}/cmake/cpack.cmake)
