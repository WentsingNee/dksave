/**
 * @file       main.cpp
 * @brief
 * @date       2022-10-06
 * @author     Peter
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
#include <vector>
#include <thread>
#include <memory>
#include <filesystem>
#include <regex>

// YAML-CPP
#include <yaml-cpp/yaml.h>



static std::filesystem::path working_dir = R"(D:\dk.test\)";
static std::chrono::milliseconds sleep_period = 48ms;

/**
 * 将 hh:mm:ss 格式的字符串转换为 std::chrono::seconds
 */
static std::chrono::seconds parse_clock(std::string const & s)
{
	static std::regex const pattern(R"(\d\d:\d\d:\d\d)");
	if (!std::regex_match(s, pattern)) {
		throw std::runtime_error("wrong clock format");
	}

	int hour = std::stoi(s.substr(0, 2));
	int min = std::stoi(s.substr(3, 2));
	int sec = std::stoi(s.substr(6, 2));
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

	std::vector<std::unique_ptr<ucamera_factory> > factories;

#if DKSAVE_ENABLE_K4A
	factories.push_back(std::make_unique<k4a_camera_factory>());
#endif

#if DKSAVE_ENABLE_OB
	factories.push_back(std::make_unique<ob_camera_factory>());
#endif

	std::vector<std::unique_ptr<ucamera> > ucameras;

	for (auto & factory : factories) {
		std::vector<std::unique_ptr<ucamera> > cameras = factory->find_cameras(config_node);
		for (auto & camera: cameras) {
			ucameras.push_back(std::move(camera));
		}
	}


	std::vector<std::thread> threads;
	threads.reserve(ucameras.size());
	for (auto & ucamera: ucameras) {
		threads.emplace_back([&ucamera]() {
			ucamera->working_loop(working_dir, sleep_period);
		});
	}

	// 等待所有线程结束
	for (std::thread & thread: threads) {
		thread.join();
	}

	KERBAL_LOG_WRITE(KINFO, "Good Bye!");

	return EXIT_SUCCESS;
} catch (std::exception const & e) {
	KERBAL_LOG_WRITE(KFATAL, "DK Save exit with exception. exception type: {}, what: {}", typeid(e).name(), e.what());
} catch (...) {
	KERBAL_LOG_WRITE(KFATAL, "DK Save exit with exception. exception type: unknown");
}
