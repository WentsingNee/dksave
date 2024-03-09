

message(STATUS "Finding fmt")
find_package(fmt REQUIRED)
message(STATUS "Finding fmt -- Found")

target_link_libraries(dksave PRIVATE fmt::fmt-header-only)
