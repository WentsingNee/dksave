
find_package(yaml-cpp REQUIRED)

message(STATUS "yaml-cpp found. lib: ${YAML_CPP_LIBRARIES}")
target_link_libraries(dksave PUBLIC yaml-cpp::yaml-cpp)
