/**
 * @file       sync_group_scheduler.hpp
 * @brief
 * @date       2024-04-10
 * @author     Wentsing Nee
 * @copyright
 *      Wentsing Nee of China Agricultural University
 *   all rights reserved
 */

#ifndef DKSAVE_SCHEDULER_SYNC_GROUP_SCHEDULER_HPP
#define DKSAVE_SCHEDULER_SYNC_GROUP_SCHEDULER_HPP

#if DKSAVE_ENABLE_OB
#	include "dksave/plugins/ob/context/H264_to_cv_mat_context.hpp"
#endif

#include "dksave/plugins/ucamera.hpp"
#include "dksave/scheduler/scheduler_base.hpp"

#include "dksave/current_time.hpp"
#include "dksave/global_settings.hpp"
#include "dksave/logger.hpp"
#include "dksave/working_status.hpp"

#include <kerbal/algorithm/modifier/fill.hpp>
#include <kerbal/container/vector.hpp>

#include <chrono>
#include <condition_variable>
#include <thread>
#include <memory>
#include <mutex>


namespace dksave
{

	template <ucamera Camera_t>
	struct sync_group_scheduler : dksave::scheduler_base<Camera_t>
	{
		private:
			using scheduler_base = dksave::scheduler_base<Camera_t>;
			using capture_loop_context = typename Camera_t::capture_loop_context;

		private:
			kerbal::container::vector<Camera_t *> cameras;
			working_status group_previous_status;
			std::string master_name;

		public:
			sync_group_scheduler(kerbal::container::vector<Camera_t> & group) :
				group_previous_status(working_status::SLEEP),
				master_name(group[0].device_name())
			{
				for (auto & camera : group) {
					cameras.emplace_back(&camera);
				}
			}

			void group_start() try
			{
				// 同步工作方式启动设备时，应先启动从机，最后启动主机
				std::size_t i = this->cameras.size();
				while (i != 0) {
					--i;
					Camera_t & camera = *this->cameras[i];
					KERBAL_LOG_WRITE(KINFO, "Starting camera. camera: {}", camera.device_name());
					try {
						camera.start();
						KERBAL_LOG_WRITE(KINFO, "Camera has been started. camera: {}", camera.device_name());
					} catch (...) {
						KERBAL_LOG_WRITE(KERROR, "Camera waked up failed. camera: {}", camera.device_name());
						throw;
					}
				}
			} catch (...) {
				this->group_stop();
				throw;
			}

			void group_stop() noexcept
			{
				KERBAL_LOG_WRITE(KINFO, "Stopping the group. master_camera: {}", master_name);
				for (std::size_t i = 0; i < this->cameras.size(); ++i) {
					Camera_t & camera = *this->cameras[i];
					KERBAL_LOG_WRITE(KINFO, "Stopping the camera. camera: {}", camera.device_name());
					camera.stop();
				}
			}

			void group_stabilize() try
			{
				for (std::size_t i = 0; i < this->cameras.size(); ++i) {
					Camera_t & camera = *this->cameras[i];
					KERBAL_LOG_WRITE(KINFO, "Stabilizing the camera. camera: {}", camera.device_name());
					try {
						camera.stabilize();
						KERBAL_LOG_WRITE(KINFO, "Camera has been stabled. camera: {}", camera.device_name());
					} catch (...) {
						KERBAL_LOG_WRITE(
							KERROR, "Stabilization process failed. camera: {}",
							camera.device_name()
						);
						throw;
					}
				}
			} catch (...) {
				this->group_stop();
				throw;
			}

			void thread(std::size_t worker_cnt = 4)
			{
				using namespace std::chrono_literals;

				kerbal::container::vector<std::thread> workers;

				std::condition_variable cv;
				std::mutex mtx;
				bool ready = false;
				bool exit = false;

				for (std::size_t i = 0; i < worker_cnt; ++i) {
					workers.emplace_back([this, i, &cv, &mtx, &ready, &exit]() {
						kerbal::container::vector<bool> capture_success_flags(cameras.size(), false);
						kerbal::container::vector<std::unique_ptr<capture_loop_context> > ctx_group;
						for (std::size_t camera_i = 0; camera_i < cameras.size(); ++camera_i) {
							ctx_group.emplace_back(std::make_unique<capture_loop_context>(cameras[camera_i]));
						}

						while (true) {
							std::unique_lock<std::mutex> lock(mtx);
							cv.wait(lock, [&ready, &exit]() {
								return exit || ready;
							});
							ready = false;
							if (exit) {
								KERBAL_LOG_WRITE(KINFO, "Worker exit. camera: {}, worker id: {}", master_name, i);
								return ;
							}

#			if DKSAVE_ENABLE_OB
							try {
#			endif
								one_capture(capture_success_flags, ctx_group, lock);
#			if DKSAVE_ENABLE_OB
							} catch (dksave::plugins_ob::H264_decode_error const & e) {
								KERBAL_LOG_WRITE(
									KDEBUG, "Color frame decode from H.264 failed, retrying immediately. camera: {}, frame_count: {}",
									master_name,
									-1/*ctx.frame_count*/
								);
								continue;
							}
#			endif

						}

					});
				}

				while (true) {
					auto start_time = std::chrono::system_clock::now();

					dksave::working_status status = dksave::get_working_status(start_time);
					Camera_t & master_camera = *this->cameras[0];

					if (status != group_previous_status) {
						KERBAL_LOG_WRITE(
							KINFO, "Camera status switched. master camera: {}, from: {}, to: {}",
							master_name, group_previous_status, status
						);
						group_previous_status = status;
					}

					if (status == dksave::working_status::SLEEP) {
						if (master_camera.enable()) {
							KERBAL_LOG_WRITE(
								KINFO, "Camera fall in sleep. master_camera: {}",
								master_name
							);
							group_stop();
						}
						std::this_thread::sleep_for(1min);
						continue;
					} else {
						if (!master_camera.enable()) {
							// 启动设备
							KERBAL_LOG_WRITE(
								KINFO, "Group is sleeping, try to wake up... . master_camera: {}",
								master_name
							);
							try {
								group_start();
								KERBAL_LOG_WRITE(
									KINFO, "Group has been started. master_camera: {}",
									master_name
								);
							} catch (...) {
								KERBAL_LOG_WRITE(
									KERROR, "Group starts failed. master_camera: {}, next try period: 30s",
									master_name
								);
								std::this_thread::sleep_for(30s);
								continue;
							}
							try {
								KERBAL_LOG_WRITE(
									KINFO, "Stabilizing the group. master_camera: {}",
									master_name
								);
								group_stabilize();
								KERBAL_LOG_WRITE(
									KINFO, "Group has been stabilized. master_camera: {}",
									master_name
								);
							} catch (...) {
								KERBAL_LOG_WRITE(
									KERROR, "Stabilizing the group failed. master_camera: {}, next try period: 30s",
									master_name
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
					KERBAL_LOG_WRITE(KDEBUG, "Notifying worker thread. master_camera: {}", master_name);
					cv.notify_one();

					std::this_thread::sleep_until(start_time + global_settings::get_sleep_period());
				}
			}

		private:

			void one_capture(
				kerbal::container::vector<bool> & capture_success_flags,
				kerbal::container::vector<std::unique_ptr<capture_loop_context> > & ctx_group,
				std::unique_lock<std::mutex> & lock
			)
			{

				for (std::size_t i = 0; i < ctx_group.size(); ++i) {
					auto & ctx = *(ctx_group[i]);
					Camera_t & camera = *(ctx.camera);
					try {
						ctx.do_capture();

						capture_success_flags[i] = true;
						KERBAL_LOG_WRITE(
							KINFO, "Capture handled success. camera: {}, frame_count: {}",
							camera.device_name(),
							ctx.frame_count
						);
					} catch (std::exception const & e) {
						KERBAL_LOG_WRITE(
							KERROR,
							"Handled exception of capture. camera: {}, exception type: {}, what: {}",
							camera.device_name(),
							typeid(e).name(), e.what()
						);
					} catch (...) {
						KERBAL_LOG_WRITE(
							KERROR, "Handled exception of capture. camera: {}, exception type: unknown",
							camera.device_name()
						);
					}
				}

				lock.unlock();

				auto capture_time = std::chrono::system_clock::now();
				std::string date = dksave::format_systime_to_date(capture_time);
				std::string timestamp = dksave::format_systime_to_datetime_milli(capture_time);

				for (std::size_t i = 0; i < ctx_group.size(); ++i) {
					if (false == capture_success_flags[i]) {
						continue;
					}

					auto & ctx = *(ctx_group[i]);
					Camera_t & camera = *(ctx.camera);
					try {

						scheduler_base::handle_rgb(ctx, date, timestamp);
						scheduler_base::handle_depth(ctx, date, timestamp);

						ctx.frame_count++;
						KERBAL_LOG_WRITE(
							KINFO, "Capture handled success. camera: {}, frame_count: {}",
							camera.device_name(),
							ctx.frame_count
						);
					} catch (std::exception const & e) {
						KERBAL_LOG_WRITE(
							KERROR, "Handled exception of capture. camera: {}, exception type: {}, what: {}",
							camera.device_name(),
							typeid(e).name(), e.what()
						);
					} catch (...) {
						KERBAL_LOG_WRITE(
							KERROR, "Handled exception of capture. camera: {}, exception type: unknown",
							camera.device_name()
						);
					}
				}
			}

	};

} // namespace dksave

#endif // DKSAVE_SCHEDULER_SYNC_GROUP_SCHEDULER_HPP
