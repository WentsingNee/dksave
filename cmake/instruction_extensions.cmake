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

macro(check_compiler_support_ie_flag ie)
    set(flag ${ARGN})
    message(STATUS "Checking compiler support ${ie} flag: ${flag}")
    CHECK_CXX_COMPILER_FLAG("${flag}" KERBAL_TEST_SUPPORT_${ie})
    message(STATUS "Checking compiler support ${ie} flag - Done")
    message(STATUS "set KERBAL_TEST_SUPPORT_${ie} = ${KERBAL_TEST_SUPPORT_${ie}}")
    if (${KERBAL_TEST_SUPPORT_${ie}})
        set(KERBAL_TEST_FLAG_${ie} ${flag})
        message(STATUS "set KERBAL_TEST_FLAGS_${ie} = ${flag}")
    endif ()
    message("")
endmacro()

set(KERBAL_TEST_IE_LIST
        mmx sse sse2 sse3 ssse3 sse4.1 sse4.2 avx avx2 avx512f
        bmi bmi2 sha
        neon sve
)

foreach (ie ${KERBAL_TEST_IE_LIST})
    if (CMAKE_CXX_COMPILER_ID MATCHES "MSVC")
        if (${ie} STREQUAL "avx512f")
            check_compiler_support_ie_flag(${ie} "/arch:AVX512")
        elseif (${ie} STREQUAL "avx2")
            check_compiler_support_ie_flag(${ie} "/arch:AVX2")
        elseif (${ie} STREQUAL "avx")
            check_compiler_support_ie_flag(${ie} "/arch:AVX")
        elseif (${ie} STREQUAL "sse2")
            check_compiler_support_ie_flag(${ie} "/arch:SSE2")
        elseif (${ie} STREQUAL "sse")
            check_compiler_support_ie_flag(${ie} "/arch:SSE")
        endif ()
    else()
        if (${ie} STREQUAL "sve")
            check_compiler_support_ie_flag(${ie} "-march=armv8-a+sve")
        elseif (${ie} STREQUAL "neon")
            check_compiler_support_ie_flag(${ie} -march=armv8-a -mfpu=neon)
            if (NOT KERBAL_TEST_SUPPORT_${ie})
                unset(KERBAL_TEST_SUPPORT_${ie} CACHE)
                message(STATUS "-march=armv8-a -mfpu=neon doesn't support, try: -march=armv8-a+simd")
                check_compiler_support_ie_flag(${ie} "-march=armv8-a+simd")
            endif ()
#        elseif (${ie} STREQUAL "mmx")
#            check_compiler_ie_support(${ie} -m${ie} -mno-sse)
        else ()
            check_compiler_support_ie_flag(${ie} -m${ie})
        endif ()
    endif ()
endforeach ()

message("\n")


macro(target_wih_ie target_name ie)
    if (${KERBAL_TEST_SUPPORT_${ie}})
        target_compile_options(${target_name} PUBLIC ${KERBAL_TEST_FLAG_${ie}})
    else ()
        message(WARNING "Compiler doesn't support ${ie}")
    endif ()
endmacro()