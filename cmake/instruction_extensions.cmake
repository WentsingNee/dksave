#
# @file       instruction_extensions.cmake
# @brief
# @date       2022-12-02
# @author     Peter
# @copyright
#      Peter of [ThinkSpirit Laboratory](http://thinkspirit.org/)
#   of [Nanjing University of Information Science & Technology](http://www.nuist.edu.cn/)
#   all rights reserved
#

include(CheckCXXCompilerFlag)

message("\n")
message(STATUS "Checking compiler's support for instruction extensions")
message("")

function(check_compiler_support_ie_flags ie)
    set(flags ${ARGN})
    message(STATUS "Checking compiler support ${ie} flags: ${flags}")
    CHECK_CXX_COMPILER_FLAG("${flags}" KERBAL_TEST_SUPPORT_${ie})
    message(STATUS "Checking compiler support ${ie} flags - Done")
    message(STATUS "set KERBAL_TEST_SUPPORT_${ie} = ${KERBAL_TEST_SUPPORT_${ie}}")
    if (${KERBAL_TEST_SUPPORT_${ie}})
        set(KERBAL_TEST_FLAGS_${ie} ${flags} PARENT_SCOPE)
        message(STATUS "set KERBAL_TEST_FLAGS_${ie} = ${flags}")
    endif ()
    message("")
endfunction()

set(KERBAL_TEST_IE_LIST
        mmx sse sse2 sse3 ssse3 sse4.1 sse4.2 avx avx2 avx512f
        bmi bmi2 sha
        neon sve
)

foreach (ie ${KERBAL_TEST_IE_LIST})
    if (CMAKE_CXX_COMPILER_ID MATCHES "MSVC")
        if (${ie} STREQUAL "avx512f")
            check_compiler_support_ie_flags(${ie} "/arch:AVX512")
        elseif (${ie} STREQUAL "avx2")
            check_compiler_support_ie_flags(${ie} "/arch:AVX2")
        elseif (${ie} STREQUAL "avx")
            check_compiler_support_ie_flags(${ie} "/arch:AVX")
        elseif (${ie} STREQUAL "sse2")
            check_compiler_support_ie_flags(${ie} "/arch:SSE2")
        elseif (${ie} STREQUAL "sse")
            check_compiler_support_ie_flags(${ie} "/arch:SSE")
        endif ()
    else()
        if (${ie} STREQUAL "mmx")
            check_compiler_support_ie_flags(${ie} -m${ie} -mno-sse)
            continue()
        endif ()
        if (${ie} STREQUAL "sve")
            check_compiler_support_ie_flags(${ie} "-march=armv8-a+sve")
            continue()
        endif ()
        if (${ie} STREQUAL "neon")
            check_compiler_support_ie_flags(${ie} -march=armv8-a -mfpu=neon)
            if (KERBAL_TEST_SUPPORT_${ie})
                continue()
            endif ()

            unset(KERBAL_TEST_SUPPORT_${ie} CACHE)
            message(STATUS "-march=armv8-a -mfpu=neon doesn't support, try: -march=armv8-a+simd")
            check_compiler_support_ie_flags(${ie} "-march=armv8-a+simd")
            continue()
        endif ()

        check_compiler_support_ie_flags(${ie} -m${ie})
    endif ()
endforeach ()

message("\n")


function(target_wih_ie target_name ie)
    if (${KERBAL_TEST_SUPPORT_${ie}})
        target_compile_options(${target_name} PUBLIC ${KERBAL_TEST_FLAGS_${ie}})
    else ()
        message(WARNING "Compiler doesn't support ${ie}")
    endif ()
endfunction()
