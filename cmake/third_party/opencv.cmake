
message(STATUS "Finding OpenCV")
find_package(OpenCV REQUIRED)
message(STATUS "Finding OpenCV -- Found")

message(STATUS "OpenCV_INCLUDE_DIRS: ${OpenCV_INCLUDE_DIRS}")
message(STATUS "OpenCV LIBS: ${OpenCV_LIBS}")

target_include_directories(dksave PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(dksave
        PRIVATE
            opencv_core
            opencv_imgproc
            opencv_imgcodecs
)
