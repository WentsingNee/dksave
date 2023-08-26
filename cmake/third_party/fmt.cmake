
option(DKSAVE_DOWNLOAD_FMT "" OFF)

message(STATUS "Finding local fmt")

if (${DKSAVE_DOWNLOAD_FMT})
    find_package(fmt REQUIRED)
else ()
    find_package(fmt)

    if (NOT ${fmt_FOUND})
        message(STATUS "Local fmt Package not found, try to use downloaded version")

        include(cmake/third_party.cmake)
        include(cmake/download_file.cmake)
        include(cmake/get_filename_from_url.cmake)

        set(FMT_DOWNLOAD_URL "https://github.com/fmtlib/fmt/releases/download/9.1.0/fmt-9.1.0.zip")
        get_filename_from_url("${FMT_DOWNLOAD_URL}" FMT_ARCHIVE_FILE)
        set(FMT_ARCHIVE_FILE "${THIRD_PARTY_ROOT}/${FMT_ARCHIVE_FILE}")

        if (NOT EXISTS ${FMT_ARCHIVE_FILE})
            download_file("${FMT_DOWNLOAD_URL}" "${FMT_ARCHIVE_FILE}")
        endif ()

        message(STATUS "Extracting ${FMT_ARCHIVE_FILE}")
        file(ARCHIVE_EXTRACT INPUT ${FMT_ARCHIVE_FILE} DESTINATION "./fmt")

        file(GLOB FMT_DIRS "${CMAKE_CURRENT_BINARY_DIR}/fmt/*")

    endif ()
endif ()


if (${fmt_FOUND})
    target_link_libraries(dksave PUBLIC fmt::fmt-header-only)
endif ()
