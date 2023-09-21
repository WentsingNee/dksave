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

#include "logger.hpp"


class DKCamera
{
		k4a::device k_device;
		std::string k_device_name;
		k4a_device_configuration_t k_config;
		bool k_enable;

	public:
		DKCamera(
			k4a::device && device,
			std::string const & device_name,
			k4a_device_configuration_t && config) :
                k_device(std::move(device)),
                k_device_name(device_name),
                k_config(std::move(config)),
                k_enable(false)
		{
		}

        k4a::device & device()
        {
            return k_device;
        }

        std::string const & device_name() const
        {
            return k_device_name;
        }

		k4a_device_configuration_t const& config() const
		{
			return k_config;
		}

		bool enable() const
		{
			return k_enable;
		}

		void start() try
		{
            k_device.start_cameras(&k_config);
            k_enable = true;
			KERBAL_LOG_WRITE(KINFO, "Start camera {}.", k_device_name);
		} catch (...) {
            k_enable = false;
            throw;
        }

		// 稳定化
		void stabilize() try
		{
			k4a::capture capture;
			int success = 0; //用来稳定，类似自动曝光
			int failed = 0; // 统计自动曝光的失败次数

			while (true) {
				bool capture_success = true;
				try {
					capture_success = k_device.get_capture(&capture);
				} catch (...) {
					capture_success = false;
				}

				if (capture_success) {
					// 跳过前 n 个（成功的数据采集）循环，用来稳定
					success++;
					KERBAL_LOG_WRITE(KINFO, "Capture several frames to give auto-exposure for {} times.", success);

					if (success >= 30) {
						KERBAL_LOG_WRITE(KINFO, "Done: auto-exposure.");
						return; // 完成相机的稳定过程
					}
				} else {
					failed++;
					KERBAL_LOG_WRITE(KWARNING, "K4A_WAIT_RESULT_TIMEOUT for {} times.", failed);

					if (failed >= 30) {
						KERBAL_LOG_WRITE(KFATAL, "Failed to give auto-exposure.", failed);
						this->stop();
						return;
					}
				}
			}
		} catch (...) {
            k_enable = false;
            throw;
        }

		void stop()
		{
            k_device.stop_cameras();
            k_enable = false;
		}

};

#endif // DKSAVE_DKCAMERA_HPP
