

message(STATUS "Finding Kerbal")
find_package(Kerbal REQUIRED)
message(STATUS "Finding Kerbal -- Found")

message(STATUS "Kerbal_DIR: ${Kerbal_DIR}")

target_link_libraries(dksave PRIVATE Kerbal::kerbal)
add_compile_definitions(KERBAL_ENABLE_MODULES_EXPORT=0)

include(Kerbal/instruction_extensions)
kerbal_ies_required(avx2)
kerbal_target_with_ies(
        TARGET dksave
        IEs avx2
        SCOPE PRIVATE
        MODE WARNING
)
