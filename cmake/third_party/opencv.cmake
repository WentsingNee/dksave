
option(DKSAVE_DOWNLOAD_OPENCV "" OFF)

message(STATUS "Finding local OpenCV")

if (NOT ${DKSAVE_DOWNLOAD_OPENCV})
    find_package(OpenCV REQUIRED)
else ()
    find_package(OpenCV)

    if (NOT ${OpenCV_FOUND})
        message(STATUS "Local OpenCV Package not found, try to use downloaded version")

        include(cmake/third_party.cmake)
        include(cmake/download_file.cmake)
        include(cmake/get_filename_from_url.cmake)

        set(OPENCV_DOWNLOAD_URL "https://github.com/opencv/opencv/releases/download/4.8.0/opencv-4.8.0-windows.exe")
        get_filename_from_url("${OPENCV_DOWNLOAD_URL}" OPENCV_ARCHIVE_FILE)
        set(OPENCV_ARCHIVE_FILE "${THIRD_PARTY_ROOT}/${OPENCV_ARCHIVE_FILE}")

        if (NOT EXISTS ${OPENCV_ARCHIVE_FILE})
            download_file("${OPENCV_DOWNLOAD_URL}" "${OPENCV_ARCHIVE_FILE}")
        endif ()

    endif ()
endif ()


if (${OpenCV_FOUND})
    target_include_directories(dksave PUBLIC ${OpenCV_INCLUDE_DIRS})
    message("OpenCV LIBS: ${OpenCV_LIBS}")
    target_link_libraries(dksave PUBLIC "${OpenCV_LIBS}")
endif ()
