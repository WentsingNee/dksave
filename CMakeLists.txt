cmake_minimum_required(VERSION 3.12)
project(DKSave CXX)

set(CMAKE_CXX_STANDARD 20)
message(STATUS "${CMAKE_CXX_PLATFORMS}")

add_executable(dksave src/dksave.cpp)
target_include_directories(dksave PUBLIC include)


# OpenCV

find_package(OpenCV REQUIRED)

if (${CMAKE_CXX_PLATFORM_ID} STREQUAL "Windows")
	target_include_directories(dksave PUBLIC ${OpenCV_INCLUDE_DIRS})
elseif (${CMAKE_CXX_PLATFORM_ID} STREQUAL "Linux")
	target_include_directories(dksave PUBLIC "/usr/include/opencv4/")
endif()

message("OpenCV LIBS: ${OpenCV_LIBS}")
target_link_libraries(dksave PUBLIC "${OpenCV_LIBS}")


# fmt

message(STATUS "Finding local fmt")
find_package(fmt QUIET)

if (NOT ${fmt_FOUND})
	message(STATUS "Local fmt not found, try to use downloaded version")

	set (FMT_DOWNLOAD_URL "https://github.com/fmtlib/fmt/releases/download/9.1.0/fmt-9.1.0.zip")
	set (FMT_ARCHIVE_FILE "${PROJECT_SOURCE_DIR}/fmt.zip")

	if (NOT EXISTS ${FMT_ARCHIVE_FILE})
		message(STATUS "Downloading ${FMT_DOWNLOAD_URL}")
		file(DOWNLOAD "${FMT_DOWNLOAD_URL}" ${FMT_ARCHIVE_FILE})
		message(STATUS "Download fmt finish")
	endif ()

	message(STATUS "Extracting ${FMT_ARCHIVE_FILE}")
	file(ARCHIVE_EXTRACT INPUT ${FMT_ARCHIVE_FILE} DESTINATION "./fmt")

	file(GLOB FMT_DIRS "${CMAKE_CURRENT_BINARY_DIR}/fmt/*")
	message(STATUS "fmt dirs: ${FMT_DIRS}")
	include_directories("${FMT_DIRS}/include")
endif ()

target_compile_definitions(dksave PUBLIC FMT_HEADER_ONLY)


# K4A

#set(K4A_DOWNLOAD_URL "https://download.microsoft.com/download/3/d/6/3d6d9e99-a251-4cf3-8c6a-8e108e960b4b/Azure%20Kinect%20SDK%201.4.1.exe")
#set(K4A_DOWNLOAD_RESULT "Azure Kinect SDK 1.4.1.exe")
#file(DOWNLOAD ${K4A_DOWNLOAD_URL} ${K4A_DOWNLOAD_RESULT})

#find_library(k4a k4a REQUIRED PATHS "C:/Program Files/Azure Kinect SDK v1.4.1/sdk/windows-desktop/amd64/release/lib/")
find_library(k4a k4a REQUIRED)
target_link_libraries(dksave PUBLIC ${k4a})

target_include_directories(dksave PUBLIC "${DKSAVE_K4A_INCLUDE_PATH}")
