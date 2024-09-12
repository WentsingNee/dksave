
if (DKSAVE_ENABLE_PCL)
    message(STATUS "Finding PCL")
    find_package(PCL REQUIRED COMPONENTS common io)
    message(STATUS "Finding PCL -- Found")

    target_link_libraries(dksave PRIVATE ${PCL_LIBRARIES})
endif ()


if (DKSAVE_ENABLE_PCL)
    add_compile_definitions(DKSAVE_ENABLE_PCL=1)
else ()
    add_compile_definitions(DKSAVE_ENABLE_PCL=0)
endif ()
