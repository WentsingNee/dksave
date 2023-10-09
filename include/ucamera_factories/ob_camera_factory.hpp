/**
 * @file       ob_camera_factory.hpp
 * @brief
 * @date       2023-09-24
 * @author     Peter
 * @copyright
 *      Peter of [ThinkSpirit Laboratory](http://thinkspirit.org/)
 *   of [Nanjing University of Information Science & Technology](http://www.nuist.edu.cn/)
 *   all rights reserved
 */

#ifndef DKSAVE_OB_CAMERA_FACTORY_HPP
#define DKSAVE_OB_CAMERA_FACTORY_HPP

#include "ucamera_factory.hpp"
#include "ucameras/ob_camera.hpp"
#include "logger.hpp"

#include <string>

#include <cstdlib>

#include <libobsensor/ObSensor.hpp>
#include <yaml-cpp/yaml.h>

#include <kerbal/container/avl_set.hpp>
#include <kerbal/container/vector.hpp>
#include <regex>


namespace dksave_ob {

	struct find_cameras_context {
		kerbal::container::vector<dksave_ob::camera> cameras;
		kerbal::container::avl_set<std::string> device_name_used;
		YAML::Node cameras_node;

		ob::Context ctx;

		static log_level ob_log_level_to_kerbal(OBLogSeverity level) {
			switch (level) {
				case OB_LOG_SEVERITY_DEBUG: {
					return KDEBUG;
				}
				case OB_LOG_SEVERITY_INFO: {
					return KINFO;
				}
				case OB_LOG_SEVERITY_WARN: {
					return KWARNING;
				}
				case OB_LOG_SEVERITY_ERROR: {
					return KERROR;
				}
				case OB_LOG_SEVERITY_FATAL: {
					return KFATAL;
				}
				default: {
					return KDEBUG;
				}
			}
		}

		find_cameras_context(
				YAML::Node const &yaml_config) :
				cameras_node(yaml_config["ob_cameras"]) {
			ctx.setLoggerSeverity(OB_LOG_SEVERITY_OFF);
			ctx.setLoggerToCallback(OB_LOG_SEVERITY_INFO, [](OBLogSeverity level, char const *message) {
				try {
					static std::regex const OB_LOG_PATTERN(R"(\[.*\]\[.*\]\[.*\]\[(.*):(\d+)\] (.*))");
					std::cmatch match;
					if (std::regex_search(message, match, OB_LOG_PATTERN)) {
						std::string file(match[1].first, match[1].second);
						std::string lines(match[2].first, match[2].second);
						int line = std::stoi(lines);
						std::string msg(match[3].first, match[3].second);
						kerbal::log::log_write(std::cout, file.c_str(), line, ob_log_level_to_kerbal(level),
											   fmt::format("{}", msg));
						return;
					}
				} catch (...) {
					// pass
				}
				KERBAL_LOG_WRITE(ob_log_level_to_kerbal(level), "{}", message);
			});
		}

		void open_local_cameras() {
			YAML::Node local_cameras_node = cameras_node["local"];

			auto device_list = ctx.queryDeviceList();

			uint32_t device_count = device_list->deviceCount(); // 发现已连接的设备数
			if (0 == device_count) {
				KERBAL_LOG_WRITE(KWARNING, "There is no local OB camera found.");
				return;
			}

			for (uint32_t device_i = 0; device_i < device_count; ++device_i) {
				KERBAL_LOG_WRITE(KINFO, "Try opening local camera. idx: {}", device_i);
				auto device = device_list->getDevice(device_i);
				auto device_info = device->getDeviceInfo();
				std::string serial_num = device_info->serialNumber();
				KERBAL_LOG_WRITE(KINFO, "Serial num of {}-th local camara is {}", device_i, serial_num);

				std::string device_name;
				try {
					YAML::Node camera_node = local_cameras_node[serial_num];
					if (camera_node.IsNull()) {
						KERBAL_LOG_WRITE(KWARNING,
										 "device_name is not specified, set default to serial_num. camera serial_num: {}",
										 serial_num);
						device_name = serial_num;
					} else {
						device_name = camera_node["device_name"].as<std::string>();
					}
				} catch (std::exception const &e) {
					KERBAL_LOG_WRITE(KFATAL, "Parse device_name failed. exception type: {}, what: {}",
									 typeid(e).name(), e.what());
					throw;
				}

				cameras.emplace_back(std::move(device), device_name);
			}
		}

		void open_network_cameras() {
			YAML::Node network_cameras_node = cameras_node["network"];
			for (auto const &e: network_cameras_node) {
				uint32_t i = cameras.size();
				// 打开设备
				try {
					auto const &k = e.first;
					auto const &v = e.second;

					std::string device_ip = k.as<std::string>();
					int port = 8090;

					KERBAL_LOG_WRITE(KINFO, "Try creating net camera. device ip: {}, port: {}", device_ip, port);
					auto device = ctx.createNetDevice(device_ip.c_str(), port);
					KERBAL_LOG_WRITE(KINFO, "Net camera created success. device id: {}, port: {}", device_ip, port);

					auto device_info = device->getDeviceInfo();

					std::string serial_num = device_info->serialNumber();
					KERBAL_LOG_WRITE(KINFO, "Serial num of camara {} is {}", i, serial_num);

					std::string device_name;
					try {
						device_name = v["device_name"].as<std::string>();
					} catch (std::exception const &e) {
						KERBAL_LOG_WRITE(KFATAL, "Parse device_name failed. exception type: {}, what: {}",
										 typeid(e).name(), e.what());
						throw;
					}

					auto uir = device_name_used.insert(device_name);
					if (!uir.insert_happen()) {
						KERBAL_LOG_WRITE(KFATAL, "Specified device_name has been occupied. device_name: {}", device_name);
						exit(EXIT_FAILURE);
					}

					KERBAL_LOG_WRITE(KINFO,
									 "Open the {}-th camera with arg: serial_num: {}, device_name: {}", i,
									 serial_num, device_name);
					cameras.emplace_back(std::move(device), device_name);
				} catch (ob::Error const &e) {
					KERBAL_LOG_WRITE(KERROR, "Camara {} open failed. exception type: {}, what: {}", i, typeid(e).name(),
									 e.getMessage());
				} catch (std::exception const &e) {
					KERBAL_LOG_WRITE(KERROR, "Camara {} open failed. exception type: {}, what: {}", i, typeid(e).name(),
									 e.what());
				} catch (...) {
					KERBAL_LOG_WRITE(KERROR, "Camara {} open failed. what: unknown exception", i);
					continue;
				}
			}
		}

	};


	class camera_factory {

		public:
			static
			kerbal::container::vector<dksave_ob::camera>
			find_cameras(YAML::Node const &yaml_config) {
				find_cameras_context ctx(yaml_config);
				ctx.open_local_cameras();
				ctx.open_network_cameras();

				if (0 == ctx.cameras.size()) {
					KERBAL_LOG_WRITE(KWARNING, "There is no OB camera opened.");
				} else {
					KERBAL_LOG_WRITE(KINFO, "OB cameras found finished. found: {}", ctx.cameras.size());
				}

				return ctx.cameras;
			}
	};

	static_assert(ucamera_factory<camera_factory>,
				  "ob_camera_factory doesn't meet the requirement of ucamera_factory");

} // namespace dksave_ob

#endif // DKSAVE_OB_CAMERA_FACTORY_HPP
