
option(DKSAVE_DOWNLOAD_K4A "download K4A" OFF)


if (DKSAVE_ENABLE_K4A)

    set(K4A_DOWNLOAD_URL "https://download.microsoft.com/download/3/d/6/3d6d9e99-a251-4cf3-8c6a-8e108e960b4b/Azure%20Kinect%20SDK%201.4.1.exe")
    set(K4A_DOWNLOAD_RESULT "Azure Kinect SDK 1.4.1.exe")

    if (DKSAVE_DOWNLOAD_K4A)
        file(DOWNLOAD ${K4A_DOWNLOAD_URL} ${K4A_DOWNLOAD_RESULT})
    endif ()

    message(STATUS "Finding libk4a")

    find_library(libk4a k4a REQUIRED)
    message(STATUS "Found libk4a: ${libk4a}")
    target_link_libraries(dksave PUBLIC "${libk4a}")

    find_path(includek4a k4a/k4a.hpp REQUIRED)
    message(STATUS "Found includek4a: ${includek4a}")
    target_include_directories(dksave PUBLIC "${includek4a}")

endif ()

if (DKSAVE_ENABLE_K4A)
    add_compile_definitions(DKSAVE_ENABLE_K4A=1)
else ()
    add_compile_definitions(DKSAVE_ENABLE_K4A=0)
endif ()
