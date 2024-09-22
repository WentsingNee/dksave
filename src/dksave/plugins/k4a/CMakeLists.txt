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
                ${CMAKE_CURRENT_SOURCE_DIR}/context/k4a_img_color_to_cv_mat_context_t.cxx
                ${CMAKE_CURRENT_SOURCE_DIR}/context/k4a_img_color_transform_to_depth_mode_context_t.cxx
                ${CMAKE_CURRENT_SOURCE_DIR}/context/k4a_img_depth_to_cv_mat_context_t.cxx
                ${CMAKE_CURRENT_SOURCE_DIR}/context/k4a_img_depth_transform_to_color_mode_context_t.cxx
                ${CMAKE_CURRENT_SOURCE_DIR}/context/k4a_img_ir_to_cv_mat_context_t.cxx
                ${CMAKE_CURRENT_SOURCE_DIR}/camera.cxx
                ${CMAKE_CURRENT_SOURCE_DIR}/camera_factory.cxx
                ${CMAKE_CURRENT_SOURCE_DIR}/config.cxx
)

if (DKSAVE_ENABLE_PCL)
    target_sources(
            dksave
            PRIVATE
                FILE_SET CXX_MODULES
                FILES
                    ${CMAKE_CURRENT_SOURCE_DIR}/context/k4a_img_depth_transform_to_point_cloud_mode_context_t.cxx
                    ${CMAKE_CURRENT_SOURCE_DIR}/context/k4a_img_point_cloud_to_pcl_point_cloud_context_t.cxx
    )
endif ()
