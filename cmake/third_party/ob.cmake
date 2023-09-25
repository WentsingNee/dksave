
option(DKSAVE_DOWNLOAD_OB "download OB" OFF)


if (DKSAVE_ENABLE_OB)
    message(STATUS "Finding libob")

    find_library(libob OrbbecSDK REQUIRED)
    message(STATUS "Found libob: ${libob}")
    target_link_libraries(dksave PUBLIC "${libob}")

    find_path(includeob libobsensor/ObSensor.hpp REQUIRED)
    message(STATUS "Found includeob: ${includeob}")
    target_include_directories(dksave PUBLIC "${includeob}")

endif ()


if (DKSAVE_ENABLE_OB)
    add_compile_definitions(DKSAVE_ENABLE_OB=1)
else ()
    add_compile_definitions(DKSAVE_ENABLE_OB=0)
endif ()
