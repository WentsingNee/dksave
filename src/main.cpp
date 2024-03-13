/**
 * @file       main.cpp
 * @brief
 * @date       2022-10-06
 * @author     Wentsing Nee
 * @copyright
 *      Wentsing Nee of China Agricultural University
 *   all rights reserved
 */

#include "logger.hpp"
#include "ucamera_factory.hpp"

#if DKSAVE_ENABLE_K4A
#	include "ucamera_factories/k4a_camera_factory.hpp"
#endif

#if DKSAVE_ENABLE_OB
#	include "ucamera_factories/ob_camera_factory.hpp"
#endif

#include "working_status.hpp"

#include <chrono>
#include <string>
#include <stdexcept>
#include <thread>
#include <memory>
#include <filesystem>
#include <type_traits>

#include <ctre.hpp>
#include <yaml-cpp/yaml.h>

#include <kerbal/container/vector.hpp>
#include <kerbal/utility/tuple.hpp>


static std::filesystem::path working_dir;
static std::chrono::milliseconds sleep_period = 48ms;

/**
 * 将 hh:mm:ss 格式的字符串转换为 std::chrono::seconds
 */
static std::chrono::seconds parse_clock(std::string const & s)
{
	constexpr const char CLOCK_PATTERN[] = R"(\d\d:\d\d:\d\d)";
	if (!ctre::match<CLOCK_PATTERN>(s)) {
		throw std::runtime_error(fmt::format("wrong clock format. expect format: <hh:mm:ss>, got: \"{}\"", s));
	}

	int hour = std::stoi(s.substr(0, 2));
	if (hour < 0 || hour >= 24) {
		throw std::runtime_error("unreasonable hour in clock");
	}
	int min = std::stoi(s.substr(3, 2));
	if (min < 0 || min >= 60) {
		throw std::runtime_error("unreasonable minute in clock");
	}
	int sec = std::stoi(s.substr(6, 2));
	if (sec < 0 || sec >= 60) {
		throw std::runtime_error("unreasonable second in clock");
	}
	return std::chrono::hours(hour) + std::chrono::minutes(min) + std::chrono::seconds(sec);
}

static void parse_global_settings(YAML::Node const & config_node)
{
	auto parse_field = [](char const * field, auto parse) {
		try {
			parse();
		} catch (YAML::InvalidNode const & e) {
			KERBAL_LOG_WRITE(KFATAL, "Parse {} failed. what: {}", field, e.what());
			throw;
		} catch (std::exception const & e) {
			KERBAL_LOG_WRITE(KFATAL, "Parse {} failed. what: {}", field, e.what());
			throw;
		}
	};

	parse_field("working_dir", [&config_node]() {
		std::string working_dir_str;
		working_dir_str = config_node["working_dir"].as<std::string>();
		working_dir = std::filesystem::path(working_dir_str);
		KERBAL_LOG_WRITE(KINFO, "Parse working_dir success. working_dir: {}", working_dir_str);
	});

	parse_field("sleep_period", [&config_node]() {
		int sleep_period_i = config_node["sleep_period"].as<int>();
		sleep_period = std::chrono::milliseconds(sleep_period_i);
		KERBAL_LOG_WRITE(KINFO, "Parse sleep_period success. sleep_period: {} ms", sleep_period.count());
	});

	parse_field("start_time", [&config_node]() {
		std::string s = config_node["start_time"].as<std::string>();
		start_time = parse_clock(s);
		KERBAL_LOG_WRITE(KINFO, "Parse start_time success. start_time: {}", s);
	});

	parse_field("end_time", [&config_node]() {
		std::string s = config_node["end_time"].as<std::string>();
		end_time = parse_clock(s);
		KERBAL_LOG_WRITE(KINFO, "Parse end_time success. end_time: {}", s);
	});

	parse_field("log_level", [&config_node]() {
		std::string s = config_node["log_level"].as<std::string>();
		KERBAL_LOG_WRITE(KINFO, "Parse log_level success. log_level: {}", s);

		if (s == "KDEBUG") {
			kerbal::log::set_log_level(KDEBUG);
			return;
		}
		if (s == "KVERBOSE") {
			kerbal::log::set_log_level(KVERBOSE);
			return;
		}
		if (s == "KINFO") {
			kerbal::log::set_log_level(KINFO);
			return;
		}
		if (s == "KWARNING") {
			kerbal::log::set_log_level(KWARNING);
			return;
		}
		if (s == "KERROR") {
			kerbal::log::set_log_level(KERROR);
			return;
		}
		if (s == "KFATAL") {
			kerbal::log::set_log_level(KFATAL);
			return;
		}

		KERBAL_LOG_WRITE(KFATAL, "Unknown log level. log_level: {}", s);
		throw std::runtime_error("Unknown log level");
	});

}


template <ucamera Camera_t>
void per_camera_working_thread(Camera_t & camera)
{
	typename Camera_t::capture_loop_context ctx(&camera, working_dir);
	while (true) {
		using namespace std::chrono_literals;

		auto start_time = std::chrono::system_clock::now();

		working_status status = get_working_status(start_time);

		if (status != camera.previous_status) {
			KERBAL_LOG_WRITE(KINFO, "Camera status switched. camera: {}, from: {}, to: {}",
							 camera.device_name(), camera.previous_status, status);
			camera.previous_status = status;
		}

		if (status == working_status::SLEEP) {
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
			std::string date = format_systime_to_date(capture_time);
			std::string timestamp = format_systime_to_datetime_milli(capture_time);

			KERBAL_LOG_WRITE(KDEBUG, "Handling color frame. camera: {}, frame_count: {}", camera.device_name(),
							 ctx.frame_count);
			try {
				ctx.handle_color(date, timestamp);
				KERBAL_LOG_WRITE(KVERBOSE, "Color frame handled success. camera: {}, frame_count: {}",
								 camera.device_name(),
								 ctx.frame_count);
#		if DKSAVE_ENABLE_OB
			} catch (H264_decode_error const &e) {
				KERBAL_LOG_WRITE(KDEBUG, "Color frame decode from H.264 failed, retrying immediately. camera: {}, frame_count: {}",
								 camera.device_name(),
								 ctx.frame_count);
				continue;
#		endif
			} catch (std::exception const &e) {
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


			KERBAL_LOG_WRITE(KDEBUG, "Handling depth frame. camera: {}, frame_count: {}", camera.device_name(),
							 ctx.frame_count);
			try {
				ctx.handle_depth(date, timestamp);
				KERBAL_LOG_WRITE(KVERBOSE, "Depth frame handled success. camera: {}, frame_count: {}",
								 camera.device_name(),
								 ctx.frame_count);
			} catch (std::exception const &e) {
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


			ctx.frame_count++;
			KERBAL_LOG_WRITE(KINFO, "Capture handled success. camera: {}, frame_count: {}", camera.device_name(),
							 ctx.frame_count);
		} catch (std::exception const &e) {
			KERBAL_LOG_WRITE(KERROR, "Handled exception of capture. camera: {}, exception type: {}, what: {}",
							 camera.device_name(),
							 typeid(e).name(), e.what());
		} catch (...) {
			KERBAL_LOG_WRITE(KERROR, "Handled exception of capture. camera: {}, exception type: unknown",
							 camera.device_name());
		}
		std::this_thread::sleep_until(start_time + sleep_period);
	}
}


int main(int argc, char * argv[]) try
{
	if (argc < 2) {
		std::cerr << fmt::format("Usage: {} config.yaml", argv[0]) << std::endl;
		exit(EXIT_FAILURE);
	}

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


	std::filesystem::path config_yaml_path = argv[1];

	KERBAL_LOG_WRITE(KINFO, "Config YAML path: {}", config_yaml_path.string());
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
	parse_global_settings(config_node);

	ucamera_factories<
			0
#if DKSAVE_ENABLE_K4A
			, dksave_k4a::camera_factory
#endif
#if DKSAVE_ENABLE_OB
			, dksave_ob::camera_factory
#endif
	> factories {};


	kerbal::utility::tuple cameras_collections = factories.transform([&config_node](auto _, ucamera_factory auto factory) {
		return factory.find_cameras(config_node);
	});

	kerbal::container::vector<std::thread> threads;
	cameras_collections.for_each([&threads](auto _, auto & cameras) {
		for (auto & camera: cameras) {
			using camera_t = std::remove_reference_t<decltype(camera)>;
			threads.emplace_back(per_camera_working_thread<camera_t>, std::ref(camera));
		}
	});

	// 等待所有线程结束
	for (std::thread & thread: threads) {
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
