
function(download_file url save_path)

    message(STATUS "Downloading ${url} to ${save_path}")
    file(DOWNLOAD "${url}" "${save_path}" SHOW_PROGRESS STATUS download_status)
    message(STATUS "Download status: ${download_status}")
    list(GET download_status 0 download_return_value)

    if (download_return_value EQUAL 0)
        message(STATUS "Download finish")
    else ()
        message(FATAL_ERROR "Download failed")
        file(REMOVE "${save_path}")
    endif ()

endfunction()
