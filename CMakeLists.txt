cmake_minimum_required(VERSION 3.12)
project(DKSave CXX)

set(CMAKE_CXX_STANDARD 20)

add_executable(dksave src/dksave.cpp)
target_include_directories(dksave PUBLIC include)

target_include_directories(dksave PUBLIC "C:/Users/A/nwq_private/open_source/fmt/include/")
target_compile_definitions(dksave PUBLIC FMT_HEADER_ONLY)

target_include_directories(dksave PUBLIC "C:/Users/A/nwq_private/open_source/Microsoft.Azure.Kinect.Sensor.1.4.1/build/native/include/")
target_link_directories(dksave PUBLIC "C:/Users/A/nwq_private/open_source/Microsoft.Azure.Kinect.Sensor.1.4.1/lib/native/amd64/release/")
target_link_libraries(dksave PUBLIC "k4a.lib")


message(${CMAKE_CXX_PLATFORM_ID})

if (${CMAKE_CXX_PLATFORM_ID} STREQUAL "Windows")
	target_include_directories(dksave PUBLIC "E:/opencv/build/include/")
	target_link_directories(dksave PUBLIC "E:/opencv/build/x64/vc15/lib/")

	message(${CMAKE_BUILD_TYPE})
	if (${CMAKE_BUILD_TYPE} STREQUAL "Debug")
		target_link_libraries(dksave PUBLIC "opencv_world420d.lib")
	else()
		target_link_libraries(dksave PUBLIC "opencv_world420.lib")
	endif()
elseif (${CMAKE_CXX_PLATFORM_ID} STREQUAL "Linux")
	target_include_directories(dksave PUBLIC "/usr/include/opencv4/")
endif()