
#include(FetchContent)
#FetchContent_Declare(
#        Kerbal
#        GIT_REPOSITORY "https://github.com/WentsingNee/Kerbal.git"
#        GIT_TAG    "main.fwd"
#)
#FetchContent_MakeAvailable(Kerbal)

message(STATUS "Finding Kerbal")
find_package(Kerbal REQUIRED)
message(STATUS "Finding Kerbal -- Found")
message(STATUS "Kerbal_DIR: ${Kerbal_DIR}")
target_link_libraries(dksave PUBLIC Kerbal::kerbal)
target_compile_options(dksave PRIVATE /Zc:__cplusplus)
add_compile_definitions(KERBAL_ENABLE_MODULES_EXPORT=0)
