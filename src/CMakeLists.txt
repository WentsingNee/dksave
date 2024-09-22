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
        dksave PRIVATE main.cpp
)
target_sources(
        dksave
            PRIVATE
            FILE_SET CXX_MODULES
            FILES
                dksave/context/image_rotate_context_t.cxx
                dksave/scheduler/mono_camera_scheduler.cxx
                dksave/scheduler/scheduler_base.cxx
                dksave/scheduler/sync_group_scheduler.cxx
                dksave/current_time.cxx
                dksave/global_settings.cxx
                dksave/logger.cxx
                dksave/registration_mode_t.cxx
                dksave/rotate_flag_t.cxx
                dksave/save_cv_mat.cxx
                dksave/working_status.cxx
)

add_subdirectory(dksave/plugins)
