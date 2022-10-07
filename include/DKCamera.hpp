/**
 * @file       DKCamera.hpp
 * @brief
 * @date       2022-10-07
 * @author     Peter
 * @copyright
 *      Peter of [ThinkSpirit Laboratory](http://thinkspirit.org/)
 *   of [Nanjing University of Information Science & Technology](http://www.nuist.edu.cn/)
 *   all rights reserved
 */

#ifndef DKSAVE_DKCAMERA_HPP
#define DKSAVE_DKCAMERA_HPP

#include <k4a/k4a.hpp>
#include <windows.h>
#include <shlwapi.h>

#include "logger.hpp"


struct DKCamera
{
		k4a::device device;
		int device_id;
		k4a_device_configuration_t config;
		bool enable = true;

		void start()
		{
			if (!enable) {
				return;
			}

			device.start_cameras(&config);
			KERBAL_LOG_WRITE(KINFO, "Start camera {}.", device_id);
		}

		// 稳定化
		void stable()
		{
			if (!enable) {
				return;
			}

			k4a::capture capture;
			int success = 0;//用来稳定，类似自动曝光
			int failed = 0;// 统计自动曝光的失败次数

			while (true) {
				if (device.get_capture(&capture)) {
					// 跳过前 n 个（成功的数据采集）循环，用来稳定
					success++;
					KERBAL_LOG_WRITE(KINFO, "Capture several frames to give auto-exposure for {} times.", success);

					if (success == 30) {
						KERBAL_LOG_WRITE(KINFO, "Done: auto-exposure.");
						return; // 完成相机的稳定过程
					}
				} else {
					failed++;
					KERBAL_LOG_WRITE(KWARNING, "K4A_WAIT_RESULT_TIMEOUT for {} times.", failed);

					if (failed == 30) {
						KERBAL_LOG_WRITE(KFATAL, "Failed to give auto-exposure.", failed);
						exit(EXIT_FAILURE);
					}
				}
			}
		}

};


inline k4a_device_configuration_t get_config_0()
{
	// 配置
	k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
	config.camera_fps = K4A_FRAMES_PER_SECOND_30;
	config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;

	config.color_resolution = K4A_COLOR_RESOLUTION_720P; //1536P
	//config.color_resolution = K4A_COLOR_RESOLUTION_1536P;

	//config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
	//config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
	config.depth_mode = K4A_DEPTH_MODE_WFOV_2X2BINNED;
	//config.depth_mode = K4A_DEPTH_MODE_WFOV_UNBINNED;

	config.synchronized_images_only = true;// ensures that depth and color images are both available in the capture

	return config;
}


inline k4a_device_configuration_t get_config_1()
{
	k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
	config.camera_fps = K4A_FRAMES_PER_SECOND_15;
	config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;

	//config.color_resolution = K4A_COLOR_RESOLUTION_720P; //1536P
	config.color_resolution = K4A_COLOR_RESOLUTION_1536P;

	//config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
	//config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
	//config.depth_mode = K4A_DEPTH_MODE_WFOV_2X2BINNED;
	config.depth_mode = K4A_DEPTH_MODE_WFOV_UNBINNED;

	config.synchronized_images_only = true;// ensures that depth and color images are both available in the capture

	return config;
}



#endif // DKSAVE_DKCAMERA_HPP
