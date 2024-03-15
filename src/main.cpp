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

#include "dksave/logger.hpp"
#include "dksave/scheduler/mono_camera_scheduler.hpp"
#include "dksave/scheduler/sync_group_scheduler.hpp"
#include "dksave/global_settings.hpp"

#include <stdexcept>
#include <thread>
#include <memory>
#include <filesystem>
#include <type_traits>

#include <cstdlib>

#include <yaml-cpp/yaml.h>

#include <kerbal/container/vector.hpp>
#include <kerbal/utility/tuple.hpp>


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
	cameras_collections.for_each([&threads](auto _, auto & cameras_by_group) {
		for (auto & group : cameras_by_group) {
			if (group.index() == 0) {
				dksave::ucamera auto & camera = std::get<0>(group);
				threads.emplace_back([&camera]() {
					using camera_t = std::remove_reference_t<decltype(camera)>;
					dksave::mono_camera_scheduler<camera_t> scheduler(camera);
					scheduler.thread();
				});
			} else {
				auto & group_ = std::get<1>(group);
				if (group_.size() < 2) {
					KERBAL_LOG_WRITE(KFATAL, "A group needs at least two cameras");
					exit(EXIT_FAILURE);
				}
				threads.emplace_back([&group_]() {
					using camera_t = std::remove_reference_t<decltype(group_)>::value_type;
					dksave::sync_group_scheduler<camera_t> scheduler(group_);
					scheduler.thread();
				});
			}
		}
	});

	// 等待所有线程结束
	KERBAL_LOG_WRITE(KINFO, "Working threads created: {}", threads.size());
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
