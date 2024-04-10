/**
 * @file       main.cpp
 * @brief
 * @date       2022-10-06
 * @author     Wentsing Nee
 * @copyright
 *      Wentsing Nee of China Agricultural University
 *   all rights reserved
 */

#if DKSAVE_ENABLE_K4A

#	include "dksave/plugins/k4a/camera_factory.hpp"

#endif

#if DKSAVE_ENABLE_OB

#	include "dksave/plugins/ob/camera_factory.hpp"

#endif

#include "dksave/plugins/ucamera_factory.hpp"

#include "dksave/global_settings.hpp"
#include "dksave/logger.hpp"
#include "dksave/working_status.hpp"

#include <chrono>
#include <string>
#include <stdexcept>
#include <thread>
#include <memory>
#include <filesystem>
#include <type_traits>

#include <cstdlib>

#include <yaml-cpp/yaml.h>

#include <kerbal/container/vector.hpp>
#include <kerbal/utility/tuple.hpp>


template <dksave::ucamera Camera_t>
void per_camera_working_thread(Camera_t & camera)
{
	typename Camera_t::capture_loop_context ctx(&camera, dksave::global_settings::get_working_dir());
	while (true) {
		using namespace std::chrono_literals;

		auto start_time = std::chrono::system_clock::now();

		dksave::working_status status = dksave::get_working_status(start_time);

		if (status != camera.previous_status) {
			KERBAL_LOG_WRITE(KINFO, "Camera status switched. camera: {}, from: {}, to: {}",
							 camera.device_name(), camera.previous_status, status);
			camera.previous_status = status;
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
					KERBAL_LOG_WRITE(KERROR, "Camera waked up failed. camera: {}", camera.device_name());
					std::this_thread::sleep_for(30s);
					continue;
				}
				try {
					camera.stabilize();
					KERBAL_LOG_WRITE(KINFO, "Camera has been stable. camera: {}", camera.device_name());
				} catch (...) {
					KERBAL_LOG_WRITE(KERROR, "Stabilization process failed. camera: {}, next try period: 30s",
									 camera.device_name());
					std::this_thread::sleep_for(30s);
					continue;
				}
			}
		}

		try {
			ctx.do_capture();

			auto capture_time = std::chrono::system_clock::now();
			std::string date = dksave::format_systime_to_date(capture_time);
			std::string timestamp = dksave::format_systime_to_datetime_milli(capture_time);

			auto handle_rgb_start = std::chrono::system_clock::now();

			KERBAL_LOG_WRITE(KDEBUG, "Handling color frame. camera: {}, frame_count: {}", camera.device_name(),
							 ctx.frame_count);
			try {
				ctx.handle_color(date, timestamp);
				KERBAL_LOG_WRITE(KVERBOSE, "Color frame handled success. camera: {}, frame_count: {}",
								 camera.device_name(),
								 ctx.frame_count);
#		if DKSAVE_ENABLE_OB
			} catch (dksave::plugins_ob::H264_decode_error const & e) {
				KERBAL_LOG_WRITE(KDEBUG, "Color frame decode from H.264 failed, retrying immediately. camera: {}, frame_count: {}",
								 camera.device_name(),
								 ctx.frame_count);
				continue;
#		endif
			} catch (std::exception const & e) {
				KERBAL_LOG_WRITE(KERROR,
								 "Color frame handled failed. camera: {}, frame_count: {}, exception type: {}, what: {}",
								 camera.device_name(),
								 ctx.frame_count,
								 typeid(e).name(),
								 e.what()
				);
			} catch (...) {
				KERBAL_LOG_WRITE(KERROR,
								 "Color frame handled failed. camera: {}, frame_count: {}, exception type: unknown",
								 camera.device_name(),
								 ctx.frame_count
				);
			}

			auto handle_rgb_end = std::chrono::system_clock::now();
			auto handle_rgb_cost = std::chrono::duration_cast<std::chrono::milliseconds>(handle_rgb_end - handle_rgb_start);
			auto handle_rgb_cost_in_milli = handle_rgb_cost.count();
			if (handle_rgb_cost_in_milli > dksave::global_settings::get_frame_handle_timeout_rgb().count()) {
				KERBAL_LOG_WRITE(KWARNING, "Handling rgb takes time: {} ms, frame cnt: {}", handle_rgb_cost_in_milli, ctx.frame_count);
			} else {
				KERBAL_LOG_WRITE(KVERBOSE, "Handling rgb takes time: {} ms, frame cnt: {}", handle_rgb_cost_in_milli, ctx.frame_count);
			}


			auto handle_depth_start = std::chrono::system_clock::now();

			KERBAL_LOG_WRITE(KDEBUG, "Handling depth frame. camera: {}, frame_count: {}", camera.device_name(),
							 ctx.frame_count);
			try {
				ctx.handle_depth(date, timestamp);
				KERBAL_LOG_WRITE(KVERBOSE, "Depth frame handled success. camera: {}, frame_count: {}",
								 camera.device_name(),
								 ctx.frame_count);
			} catch (std::exception const & e) {
				KERBAL_LOG_WRITE(KERROR,
								 "Depth frame handled failed. camera: {}, frame_count: {}, exception type: {}, what: {}",
								 camera.device_name(),
								 ctx.frame_count,
								 typeid(e).name(),
								 e.what()
				);
			} catch (...) {
				KERBAL_LOG_WRITE(KERROR,
								 "Depth frame handled failed. camera: {}, frame_count: {}, exception type: unknown",
								 camera.device_name(),
								 ctx.frame_count
				);
			}

			auto handle_depth_end = std::chrono::system_clock::now();
			auto handle_depth_cost = std::chrono::duration_cast<std::chrono::milliseconds>(handle_depth_end - handle_depth_start);
			auto handle_depth_cost_in_milli = handle_depth_cost.count();
			if (handle_depth_cost_in_milli > dksave::global_settings::get_frame_handle_timeout_depth().count()) {
				KERBAL_LOG_WRITE(KWARNING, "Handling depth takes time: {} ms, frame cnt: {}", handle_depth_cost_in_milli, ctx.frame_count);
			} else {
				KERBAL_LOG_WRITE(KVERBOSE, "Handling depth takes time: {} ms, frame cnt: {}", handle_depth_cost_in_milli, ctx.frame_count);
			}


			ctx.frame_count++;
			KERBAL_LOG_WRITE(KINFO, "Capture handled success. camera: {}, frame_count: {}", camera.device_name(),
							 ctx.frame_count);
		} catch (std::exception const & e) {
			KERBAL_LOG_WRITE(KERROR, "Handled exception of capture. camera: {}, exception type: {}, what: {}",
							 camera.device_name(),
							 typeid(e).name(), e.what());
		} catch (...) {
			KERBAL_LOG_WRITE(KERROR, "Handled exception of capture. camera: {}, exception type: unknown",
							 camera.device_name());
		}
		std::this_thread::sleep_until(start_time + dksave::global_settings::get_sleep_period());
	}
}


int main(int argc, char * argv[]) try
{
	std::cout << fmt::format("Built time: {}-{}", __DATE__, __TIME__) << std::endl;

#if DKSAVE_ENABLE_K4A
	std::cout << fmt::format("DKSAVE_ENABLE_K4A: {}", true) << std::endl;
#else
	std::cout << fmt::format("DKSAVE_ENABLE_K4A: {}", false) << std::endl;
#endif

#if DKSAVE_ENABLE_OB
	std::cout << fmt::format("DKSAVE_ENABLE_OB: {}", true) << std::endl;
#else
	std::cout << fmt::format("DKSAVE_ENABLE_OB: {}", false) << std::endl;
#endif

	if (argc < 2) {
		std::cerr << fmt::format("Usage: {} config.yaml", argv[0]) << std::endl;
		exit(EXIT_FAILURE);
	}


	std::filesystem::path config_yaml_path = argv[1];

	KERBAL_LOG_WRITE(KINFO, "Config YAML path: {}", config_yaml_path.string());
	if (!std::filesystem::exists(config_yaml_path)) {
		KERBAL_LOG_WRITE(KFATAL, "Config YAML file not found");
		exit(EXIT_FAILURE);
	}
	YAML::Node yaml_root_node;
	try {
		yaml_root_node = YAML::LoadFile(config_yaml_path.string());
	} catch (YAML::BadFile const & e) {
		KERBAL_LOG_WRITE(KFATAL, "DK config yaml cannot be loaded. what: {}", e.what());
		throw;
	} catch (YAML::ParserException const & e) {
		KERBAL_LOG_WRITE(KFATAL, "DK config yaml has format error. what: {}", e.what());
		throw;
	}

	YAML::Node config_node = yaml_root_node["config"];
	dksave::global_settings::parse(config_node);

	dksave::ucamera_factories<
		0
#if DKSAVE_ENABLE_K4A
		, dksave::plugins_k4a::camera_factory
#endif
#if DKSAVE_ENABLE_OB
		, dksave::plugins_ob::camera_factory
#endif
	> factories{};


	kerbal::utility::tuple cameras_collections = factories.transform([&config_node](auto _, dksave::ucamera_factory auto factory) {
		return factory.find_cameras(config_node);
	});

	kerbal::container::vector<std::thread> threads;
	cameras_collections.for_each([&threads](auto _, auto & cameras) {
		for (auto & camera : cameras) {
			using camera_t = std::remove_reference_t<decltype(camera)>;
			threads.emplace_back(per_camera_working_thread<camera_t>, std::ref(camera));
		}
	});

	// 等待所有线程结束
	for (std::thread & thread : threads) {
		thread.join();
	}

	KERBAL_LOG_WRITE(KINFO, "Good Bye!");

	KERBAL_LOG_WRITE(KDEBUG, "DEBUG");
	KERBAL_LOG_WRITE(KVERBOSE, "VERBOSE");
	KERBAL_LOG_WRITE(KINFO, "INFO");
	KERBAL_LOG_WRITE(KWARNING, "WARNING");
	KERBAL_LOG_WRITE(KERROR, "ERROR");
	KERBAL_LOG_WRITE(KFATAL, "FATAL");


	return EXIT_SUCCESS;
} catch (std::exception const & e) {
	KERBAL_LOG_WRITE(KFATAL, "DK Save exit with exception. exception type: {}, what: {}", typeid(e).name(), e.what());
} catch (...) {
	KERBAL_LOG_WRITE(KFATAL, "DK Save exit with exception. exception type: unknown");
}
