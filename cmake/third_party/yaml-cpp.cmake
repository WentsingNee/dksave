

message(STATUS "Finding yaml-cpp")
find_package(yaml-cpp REQUIRED)
message(STATUS "Finding yaml-cpp -- Found")

message(STATUS "YAML_CPP_LIBRARIES: ${YAML_CPP_LIBRARIES}")

target_link_libraries(dksave PRIVATE yaml-cpp::yaml-cpp)
