#
# @file       CMakeLists.txt
# @brief
# @date       2024-09-22
# @author     Peter
# @copyright
#      Peter of [ThinkSpirit Laboratory](http://thinkspirit.org/)
#   of [Nanjing University of Information Science & Technology](http://www.nuist.edu.cn/)
#   all rights reserved
#

target_sources(
        dksave
        PRIVATE
            FILE_SET CXX_MODULES
            FILES
                ${CMAKE_CURRENT_SOURCE_DIR}/ucamera.cxx
                ${CMAKE_CURRENT_SOURCE_DIR}/ucamera_factory.cxx
)

if (DKSAVE_ENABLE_K4A)
    add_subdirectory(./k4a)
endif ()

if (DKSAVE_ENABLE_OB)
    add_subdirectory(./ob)
endif ()
