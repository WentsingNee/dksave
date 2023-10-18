
message(STATUS "Finding local OpenCV")

find_package(OpenCV REQUIRED)


target_include_directories(dksave PUBLIC ${OpenCV_INCLUDE_DIRS})
message("OpenCV LIBS: ${OpenCV_LIBS}")
target_link_libraries(dksave PRIVATE "${OpenCV_LIBS}")
