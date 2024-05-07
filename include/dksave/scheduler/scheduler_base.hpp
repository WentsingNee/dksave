/**
 * @file       scheduler_base.hpp
 * @brief
 * @date       2024-04-10
 * @author     Wentsing Nee
 * @copyright
 *      Wentsing Nee of China Agricultural University
 *   all rights reserved
 */

#ifndef DKSAVE_SCHEDULER_SCHEDULER_BASE_HPP
#define DKSAVE_SCHEDULER_SCHEDULER_BASE_HPP

#include "dksave/plugins/ucamera.hpp"

#include "dksave/global_settings.hpp"
#include "dksave/logger.hpp"
#include "dksave/working_status.hpp"

#include <chrono>
#include <filesystem>
#include <thread>


namespace dksave
{

	template <dksave::ucamera Camera_t>
	struct scheduler_base
	{

			static
			void handle_rgb(typename Camera_t::capture_loop_context & ctx, std::string const & date, std::string const & timestamp)
			{
				Camera_t & camera = *(ctx.camera);
				auto handle_rgb_start = std::chrono::system_clock::now();

				KERBAL_LOG_WRITE(
					KDEBUG, "Handling color frame. camera: {}, frame_count: {}",
					camera.device_name(),
					ctx.frame_count
				);
				try {
					std::filesystem::path camera_working_dir(dksave::global_settings::get_working_dir() / camera.device_name());
					std::filesystem::path filename_color = camera_working_dir / "rgb" / date / (timestamp + ".png");
					ctx.handle_color(filename_color);
					KERBAL_LOG_WRITE(
						KVERBOSE, "Color frame handled success. camera: {}, frame_count: {}",
						camera.device_name(),
						ctx.frame_count
					);
				} catch (std::exception const & e) {
					KERBAL_LOG_WRITE(
						KERROR, "Color frame handled failed. camera: {}, frame_count: {}, exception type: {}, what: {}",
						camera.device_name(),
						ctx.frame_count,
						typeid(e).name(), e.what()
					);
				} catch (...) {
					KERBAL_LOG_WRITE(
						KERROR, "Color frame handled failed. camera: {}, frame_count: {}, exception type: unknown",
						camera.device_name(),
						ctx.frame_count
					);
				}

				auto handle_rgb_end = std::chrono::system_clock::now();
				auto handle_rgb_cost = std::chrono::duration_cast<std::chrono::milliseconds>(handle_rgb_end - handle_rgb_start);
				auto handle_rgb_cost_in_milli = handle_rgb_cost.count();
				if (handle_rgb_cost_in_milli > dksave::global_settings::get_frame_handle_timeout_rgb().count()) {
					KERBAL_LOG_WRITE(
						KWARNING, "Handling rgb takes time: {} ms, camera: {}, frame cnt: {}",
						handle_rgb_cost_in_milli, camera.device_name(), ctx.frame_count
					);
				} else {
					KERBAL_LOG_WRITE(
						KVERBOSE, "Handling rgb takes time: {} ms, camera: {}, frame cnt: {}",
						handle_rgb_cost_in_milli, camera.device_name(), ctx.frame_count
					);
				}

			}

			static
			void handle_depth(typename Camera_t::capture_loop_context & ctx, std::string const & date, std::string const & timestamp)
			{
				Camera_t & camera = *(ctx.camera);
				auto handle_depth_start = std::chrono::system_clock::now();

				KERBAL_LOG_WRITE(
					KDEBUG, "Handling depth frame. camera: {}, frame_count: {}",
					camera.device_name(),
					ctx.frame_count
				);
				try {
					std::filesystem::path camera_working_dir(dksave::global_settings::get_working_dir() / camera.device_name());
					std::filesystem::path filename_depth = camera_working_dir / "depth" / date / (timestamp + ".png");
					std::filesystem::path filename_pcloud = camera_working_dir / "clouds" / date / (timestamp + ".ply");
					ctx.handle_depth(filename_depth, filename_pcloud);
					KERBAL_LOG_WRITE(
						KVERBOSE, "Depth frame handled success. camera: {}, frame_count: {}",
						camera.device_name(),
						ctx.frame_count
					);
				} catch (std::exception const & e) {
					KERBAL_LOG_WRITE(
						KERROR, "Depth frame handled failed. camera: {}, frame_count: {}, exception type: {}, what: {}",
						camera.device_name(),
						ctx.frame_count,
						typeid(e).name(), e.what()
					);
				} catch (...) {
					KERBAL_LOG_WRITE(
						KERROR, "Depth frame handled failed. camera: {}, frame_count: {}, exception type: unknown",
						camera.device_name(),
						ctx.frame_count
					);
				}

				auto handle_depth_end = std::chrono::system_clock::now();
				auto handle_depth_cost = std::chrono::duration_cast<std::chrono::milliseconds>(handle_depth_end - handle_depth_start);
				auto handle_depth_cost_in_milli = handle_depth_cost.count();
				if (handle_depth_cost_in_milli > dksave::global_settings::get_frame_handle_timeout_depth().count()) {
					KERBAL_LOG_WRITE(
						KWARNING, "Handling depth takes time: {} ms, camera: {}, frame cnt: {}",
						handle_depth_cost_in_milli, camera.device_name(), ctx.frame_count
					);
				} else {
					KERBAL_LOG_WRITE(
						KVERBOSE, "Handling depth takes time: {} ms, camera: {}, frame cnt: {}",
						handle_depth_cost_in_milli, camera.device_name(), ctx.frame_count
					);
				}

			}
	};

} // namespace dksave

#endif // DKSAVE_SCHEDULER_SCHEDULER_BASE_HPP
