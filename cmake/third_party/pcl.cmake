
option(DKSAVE_DOWNLOAD_PCL "download pcl library" OFF)


if (DKSAVE_ENABLE_PCL)

    message(STATUS "Finding local pcl")

    if (NOT ${DKSAVE_DOWNLOAD_PCL})
        find_package(PCL REQUIRED)
    else ()
        find_package(PCL)

        if (NOT ${PCL_FOUND})
            message(STATUS "Local pcl not found, try to use downloaded version")

            include(cmake/third_party.cmake)
            include(cmake/download_file.cmake)
            include(cmake/get_filename_from_url.cmake)

            set(PCL_DOWNLOAD_URL "https://github.com/PointCloudLibrary/pcl/releases/download/pcl-1.13.1/PCL-1.13.1-AllInOne-msvc2022-win64.exe")
            get_filename_from_url("${PCL_DOWNLOAD_URL}" PCL_INSTALL_FILE)
            set(PCL_INSTALL_FILE "${THIRD_PARTY_ROOT}/${PCL_INSTALL_FILE}")

            if (NOT EXISTS ${PCL_ARCHIVE_FILE})
                download_file("${PCL_DOWNLOAD_URL}" "${PCL_INSTALL_FILE}")
            endif ()

        endif ()
    endif ()


    if (${PCL_FOUND})
        target_include_directories(dksave PUBLIC ${PCL_INCLUDE_DIRS})
        target_link_libraries(dksave PUBLIC ${PCL_LIBRARIES})
    endif ()

endif ()
