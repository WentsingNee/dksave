

message(STATUS "Finding fmt")
find_package(fmt REQUIRED)
message(STATUS "Finding fmt -- Found")

target_link_libraries(dksave PRIVATE fmt::fmt)
#target_compile_definitions(dksave PRIVATE FMT_MODULE=ON)
