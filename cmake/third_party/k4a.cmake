
if (DKSAVE_ENABLE_K4A)
    message(STATUS "Finding k4a")
    find_package(k4a REQUIRED)
    message(STATUS "Finding k4a -- Found")

    target_link_libraries(dksave PRIVATE k4a::k4a)
endif ()


if (DKSAVE_ENABLE_K4A)
    add_compile_definitions(DKSAVE_ENABLE_K4A=1)
else ()
    add_compile_definitions(DKSAVE_ENABLE_K4A=0)
endif ()
