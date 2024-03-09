
option(DKSAVE_DOWNLOAD_OB "download OB" OFF)


if (DKSAVE_ENABLE_OB)
    message(STATUS "Finding libob")
    find_package(OrbbecSDK REQUIRED)
    message(STATUS "Finding libob -- Found")

    target_link_libraries(dksave PRIVATE OrbbecSDK::OrbbecSDK)
endif ()


if (DKSAVE_ENABLE_OB)
    add_compile_definitions(DKSAVE_ENABLE_OB=1)
else ()
    add_compile_definitions(DKSAVE_ENABLE_OB=0)
endif ()
