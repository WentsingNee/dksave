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
#	include "dksave/plugins/ob/context/H264_to_cv_mat_context.hpp"
#endif

#include "dksave/plugins/ucamera.hpp"
#include "dksave/scheduler/scheduler_base.hpp"

#include "dksave/current_time.hpp"
#include "dksave/global_settings.hpp"
#include "dksave/logger.hpp"
#include "dksave/working_status.hpp"

#include <chrono>
#include <thread>
#include <atomic>
#include <condition_variable>

#include <kerbal/container/vector.hpp>


namespace dksave
{

	template <dksave::ucamera Camera_t>
	struct mono_camera_scheduler : dksave::scheduler_base<Camera_t>
	{
		private:
			using scheduler_base = dksave::scheduler_base<Camera_t>;

		private:
			Camera_t * const camera;
			working_status previous_status;

		public:
			mono_camera_scheduler(Camera_t & camera) :
				camera(&camera),
				previous_status(working_status::WORK)
			{
			}

			void thread(std::size_t worker_cnt = 4)
			{
				kerbal::container::vector<std::thread> workers;

				std::condition_variable cv;
				std::mutex mtx;
				bool ready = false;
				bool exit = false;

				for (std::size_t i = 0; i < worker_cnt; ++i) {
					workers.emplace_back([this, i, &cv, &mtx, &ready, &exit]() {
						typename Camera_t::capture_loop_context ctx(camera);
						while (true) {
							std::unique_lock<std::mutex> lock(mtx);
							cv.wait(lock, [&ready, &exit]() {
								return exit || ready;
							});
							ready = false;
							if (exit) {
								KERBAL_LOG_WRITE(KINFO, "Worker exit. camera: {}, worker id: {}", camera->device_name(), i);
								return ;
							}

							one_capture(ctx, lock);
						}
					});
				}

				Camera_t & camera = *(this->camera);
				while (!exit) {
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

					{
						std::lock_guard guard(mtx);
						ready = true;
					}
					KERBAL_LOG_WRITE(KDEBUG, "Notifying worker thread. camera: {}", camera.device_name());
					cv.notify_one();

					KERBAL_LOG_WRITE(KDEBUG, "Start sleep. camera: {}", camera.device_name());
					std::this_thread::sleep_until(start_time + dksave::global_settings::get_sleep_period());
					KERBAL_LOG_WRITE(KDEBUG, "End sleep. camera: {}", camera.device_name());
				}

				for (auto & worker : workers) {
					worker.join();
				}
			}

			static
			void one_capture(typename Camera_t::capture_loop_context & ctx, std::unique_lock<std::mutex> & lock)
			{
				Camera_t & camera = *(ctx.camera);
				try {
					ctx.do_capture();
					lock.unlock();

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
