/**
 * @file       mono_camera_scheduler.hpp
 * @brief
 * @date       2024-04-10
 * @author     Wentsing Nee
 * @copyright
 *      Wentsing Nee of China Agricultural University
 *   all rights reserved
 */

#ifndef DKSAVE_SCHEDULER_MONO_CAMERA_SCHEDULER_HPP
#define DKSAVE_SCHEDULER_MONO_CAMERA_SCHEDULER_HPP

#if DKSAVE_ENABLE_OB
#	include "dksave/plugins/ob/context/H264_to_cv_mat_context_t.hpp"
#endif

#include "dksave/plugins/ucamera.hpp"
#include "dksave/scheduler/scheduler_base.hpp"

#include "dksave/current_time.hpp"
#include "dksave/global_settings.hpp"
#include "dksave/logger.hpp"
#include "dksave/working_status.hpp"

#include <chrono>
#include <thread>


namespace dksave
{

	template <dksave::ucamera Camera_t>
	struct mono_camera_scheduler : dksave::scheduler_base<Camera_t>
	{
		private:
			using scheduler_base = dksave::scheduler_base<Camera_t>;

		private:
			typename Camera_t::capture_loop_context ctx;
			working_status previous_status;

		public:
			mono_camera_scheduler(Camera_t & camera) :
				ctx(&camera),
				previous_status(working_status::WORK)
			{
			}

			void thread()
			{
				Camera_t & camera = *(ctx.camera);
				while (true) {
					using namespace std::chrono_literals;

					auto start_time = std::chrono::system_clock::now();

					dksave::working_status status = dksave::get_working_status(start_time);

					if (status != previous_status) {
						KERBAL_LOG_WRITE(
							KINFO, "Camera status switched. camera: {}, from: {}, to: {}",
							camera.device_name(), previous_status, status
						);
						previous_status = status;
					}

					if (status == dksave::working_status::SLEEP) {
						if (camera.enable()) {
							KERBAL_LOG_WRITE(KINFO, "Camera fall in sleep. camera: {}", camera.device_name());
							camera.stop();
						}
						std::this_thread::sleep_for(1min);
						continue;
					} else {
						if (!camera.enable()) {
							// 启动设备
							KERBAL_LOG_WRITE(KINFO, "Camera is sleeping, try to wake up... . camera: {}", camera.device_name());
							try {
								camera.start();
								KERBAL_LOG_WRITE(KINFO, "Camera has been started. camera: {}", camera.device_name());
							} catch (...) {
								KERBAL_LOG_WRITE(KERROR, "Camera waked up failed. camera: {}, next try period: 30s", camera.device_name());
								std::this_thread::sleep_for(30s);
								continue;
							}
							try {
								KERBAL_LOG_WRITE(
									KINFO, "Stabilizing the camera. camera: {}",
									camera.device_name()
								);
								camera.stabilize();
								KERBAL_LOG_WRITE(KINFO, "Camera has been stabled. camera: {}", camera.device_name());
							} catch (...) {
								KERBAL_LOG_WRITE(
									KERROR, "Stabilization process failed. camera: {}, next try period: 30s",
									camera.device_name()
								);
								std::this_thread::sleep_for(30s);
								continue;
							}
						}
					}

#		if DKSAVE_ENABLE_OB
					try {
#		endif
						one_capture();
#		if DKSAVE_ENABLE_OB
					} catch (dksave::plugins_ob::H264_decode_error const & e) {
						KERBAL_LOG_WRITE(
							KDEBUG, "Color frame decode from H.264 failed, retrying immediately. camera: {}, frame_count: {}",
							camera.device_name(),
							ctx.frame_count
						);
						continue;
					}
#		endif

					std::this_thread::sleep_until(start_time + dksave::global_settings::get_sleep_period());
				}
			}

			void one_capture()
			{
				Camera_t & camera = *(ctx.camera);
				try {
					ctx.do_capture();

					auto capture_time = std::chrono::system_clock::now();
					std::string date = dksave::format_systime_to_date(capture_time);
					std::string timestamp = dksave::format_systime_to_datetime_milli(capture_time);

					scheduler_base::handle_rgb(ctx, date, timestamp);
					scheduler_base::handle_depth(ctx, date, timestamp);

					ctx.frame_count++;
					KERBAL_LOG_WRITE(
						KINFO, "Capture handled success. camera: {}, frame_count: {}",
						camera.device_name(), ctx.frame_count
					);
				} catch (std::exception const & e) {
					KERBAL_LOG_WRITE(
						KERROR, "Capture handled failed. camera: {}, exception type: {}, what: {}",
						camera.device_name(),
						typeid(e).name(), e.what()
					);
				} catch (...) {
					KERBAL_LOG_WRITE(
						KERROR, "Capture handled failed. camera: {}, exception type: unknown",
						camera.device_name()
					);
				}
			}

	};

} // namespace dksave

#endif // DKSAVE_SCHEDULER_MONO_CAMERA_SCHEDULER_HPP
