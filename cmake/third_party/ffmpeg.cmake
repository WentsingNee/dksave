
if (DKSAVE_ENABLE_OB)
    message(STATUS "Finding ffmpeg")
    find_package(FFMPEG REQUIRED)
    message(STATUS "Finding ffmpeg -- Found")

    message(STATUS "FFMPEG_INCLUDE_DIRS: ${FFMPEG_INCLUDE_DIRS}")
    message(STATUS "FFMPEG_LIBRARY_DIRS: ${FFMPEG_LIBRARY_DIRS}")
    message(STATUS "FFMPEG_LIBRARIES: ${FFMPEG_LIBRARIES}")

    target_include_directories(dksave PRIVATE ${FFMPEG_INCLUDE_DIRS})
    target_link_directories(dksave PRIVATE ${FFMPEG_LIBRARY_DIRS})
#    target_link_libraries(dksave PRIVATE ${FFMPEG_LIBRARIES})
    target_link_libraries(dksave PRIVATE
#            avdevice
#            avfilter
#            avformat
            avcodec
            swresample
            swscale
            avutil
    )
endif ()
