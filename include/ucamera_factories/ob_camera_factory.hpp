/**
 * @file       ob_camera_factory.hpp
 * @brief
 * @date       2023-09-24
 * @author     Wentsing Nee
 * @copyright
 *      Wentsing Nee of China Agricultural University
 *   all rights reserved
 */

#ifndef DKSAVE_OB_CAMERA_FACTORY_HPP
#define DKSAVE_OB_CAMERA_FACTORY_HPP

#include "config/ob_config.hpp"
#include "ucamera_factory.hpp"
#include "ucameras/ob_camera.hpp"
#include "logger.hpp"

#include <iostream>
#include <stdexcept>
#include <string>

#include <cstdlib>

#include <ctre.hpp>
#include <libobsensor/ObSensor.hpp>
#include <fmt/format.h>
#include <yaml-cpp/yaml.h>

#include <kerbal/container/avl_set.hpp>
#include <kerbal/container/vector.hpp>


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

		static ob_camera_configuration yaml_to_config(YAML::Node const &config_yaml)
		{
			ob_camera_configuration config{};

			{
				std::string s;
				try {
					s = config_yaml["width"].as<std::string>();
				} catch (YAML::InvalidNode const &e) {
					KERBAL_LOG_WRITE(KERROR, "Invalid node. key: {}, what: {}", "width", e.what());
					throw;
				}
				try {
					config.width = std::stoi(s);
				} catch (std::exception const & e) {
					KERBAL_LOG_WRITE(KERROR, "Invalid width value. width: {}, what: {}", s, e.what());
					throw;
				}
			}
			{
				std::string s;
				try {
					s = config_yaml["height"].as<std::string>();
				} catch (YAML::InvalidNode const &e) {
					KERBAL_LOG_WRITE(KERROR, "Invalid node. key: {}, what: {}", "height", e.what());
					throw;
				}
				try {
					config.height = std::stoi(s);
				} catch (std::exception const & e) {
					KERBAL_LOG_WRITE(KERROR, "Invalid height value. height: {}, what: {}", s, e.what());
					throw;
				}
			}
			{
				std::string s;
				try {
					s = config_yaml["format"].as<std::string>();
				} catch (YAML::InvalidNode const &e) {
					KERBAL_LOG_WRITE(KERROR, "Invalid node. key: {}, what: {}", "format", e.what());
					throw;
				}
				try {
					config.format = dksave_ob::str_to_OB_FORMAT(s.c_str());
				} catch (std::exception const & e) {
					KERBAL_LOG_WRITE(KERROR, "Invalid OB format. format: {}, what: {}", s, e.what());
					throw;
				}
				if (
						config.format != OB_FORMAT_RGB &&
						config.format != OB_FORMAT_H264 &&
						config.format != OB_FORMAT_Y16
				) {
					std::string msg = fmt::format("This OB format is not supported now. format: {}", config.format);
					KERBAL_LOG_WRITE(KERROR, "{}", msg);
					throw std::runtime_error(msg);
				}
			}
			{
				std::string s;
				try {
					s = config_yaml["fps"].as<std::string>();
				} catch (YAML::InvalidNode const &e) {
					KERBAL_LOG_WRITE(KERROR, "Invalid node. key: {}, what: {}", "fps", e.what());
					throw;
				}
				try {
					config.fps = std::stoi(s);
				} catch (std::exception const & e) {
					KERBAL_LOG_WRITE(KERROR, "Invalid fps value. fps: {}, what: {}", s, e.what());
					throw;
				}
			}

			return config;
		}


		find_cameras_context(
				YAML::Node const &yaml_config) :
				cameras_node(yaml_config["ob_cameras"]
		) {
			ctx.setLoggerSeverity(OB_LOG_SEVERITY_OFF);
			ctx.setLoggerToCallback(OB_LOG_SEVERITY_INFO, [](OBLogSeverity level, char const *message) {
				try {
					auto result = ctre::match<R"(\[.*\]\[.*\]\[.*\]\[(.*):(\d+)\] (.*))">(message);
					if (result) {
						std::string file(result.get<1>());
						int line = std::stoi(result.get<2>().to_string());
						std::string msg(result.get<3>());
						kerbal::log::log_write(std::cout, file.c_str(), line, ob_log_level_to_kerbal(level), std::move(msg));
						return;
					}
				} catch (...) {
					// pass
				}
				KERBAL_LOG_WRITE(ob_log_level_to_kerbal(level), "{}", message);
			});
		}

		void config_camera(std::shared_ptr<ob::Device> device, YAML::Node const & camera_node, std::string const & deployment, std::string const & label) {
			std::string device_name;
			YAML::Node device_name_node = camera_node["device_name"];
			try {
				device_name = device_name_node.as<std::string>();
				KERBAL_LOG_WRITE(KINFO,
								 "Binding device_name with camera success. deployment: {}, label: {}, device_name: {}",
								 deployment, label, device_name);
			} catch (std::exception const &e) {
				device_name = fmt::format("ob_{}_{}", deployment, label);
				KERBAL_LOG_WRITE(KERROR, "Parse device_name failed. deployment: {}, label: {}, exception type: {}, what: {}",
								 deployment, label, typeid(e).name(), e.what());
				KERBAL_LOG_WRITE(KWARNING, "Bind default device_name with camera. deployment: {}, label: {}, device_name: {}",
								 deployment, label, device_name);
			}

			auto uir = device_name_used.insert(device_name);
			if (!uir.insert_happen()) {
				KERBAL_LOG_WRITE(KFATAL, "Specified device_name has been occupied. device_name: {}", device_name);
				exit(EXIT_FAILURE);
			}

			YAML::Node config_rgb_node = camera_node["config_rgb"];
			ob_camera_configuration config_rgb{};
			try {
				config_rgb = yaml_to_config(config_rgb_node);
			} catch (std::exception const &e) {
				KERBAL_LOG_WRITE(KERROR, "Parse config_rgb failed. device_name: {}, exception type: {}, what: {}",
								 device_name, typeid(e).name(), e.what());
				throw;
			}

			YAML::Node config_depth_node = camera_node["config_depth"];
			ob_camera_configuration config_depth{};
			try {
				config_depth = yaml_to_config(config_depth_node);
			} catch (std::exception const &e) {
				KERBAL_LOG_WRITE(KERROR, "Parse config_depth failed. device_name: {}, exception type: {}, what: {}",
								 device_name,  typeid(e).name(), e.what());
				throw;
			}

			KERBAL_LOG_WRITE(KINFO, "Appending camera into cameras list. device_name: {}", device_name);
			cameras.emplace_back(std::move(device), device_name, config_rgb, config_depth);
			KERBAL_LOG_WRITE(KINFO, "Appending camera into cameras list success. device_name: {}", device_name);
		}

		void open_local_cameras() {
			YAML::Node local_cameras_node = cameras_node["local"];

			auto device_list = ctx.queryDeviceList();

			uint32_t device_count = device_list->deviceCount(); // 发现已连接的设备数
			if (0 == device_count) {
				KERBAL_LOG_WRITE(KWARNING, "There is no local OB camera found.");
				return;
			}

			for (uint32_t i = 0; i < device_count; ++i) {
				KERBAL_LOG_WRITE(KINFO, "Try opening local camera. index: {}", i);
				auto device = device_list->getDevice(i);
				auto device_info = device->getDeviceInfo();
				std::string serial_num = device_info->serialNumber();

				KERBAL_LOG_WRITE(KINFO, "Serial number of camara is fetched. deployment: {}, serial_num: {}", "local", serial_num);

				YAML::Node camera_node = local_cameras_node[serial_num];
				if (!camera_node.IsDefined()) {
					KERBAL_LOG_WRITE(KERROR,
									 "Configuration of the camera is not specified, this camera will be ignored. deployment: {}, serial_num: {}",
									 "local", serial_num);
					continue;
				}

				config_camera(device, camera_node, "local", serial_num);
			}
		}

		void open_network_cameras() {
			YAML::Node network_cameras_node = cameras_node["network"];
			for (auto const &e: network_cameras_node) {
				auto const &k = e.first;
				auto const &v = e.second;

				std::string device_ip = k.as<std::string>();
				int port = 8090;

				KERBAL_LOG_WRITE(KINFO, "Creating network camera. device_ip: {}, port: {}", device_ip, port);
				// 打开设备
				std::shared_ptr<ob::Device> device;
				try {
					device = ctx.createNetDevice(device_ip.c_str(), port);
				} catch (ob::Error const &e) {
					KERBAL_LOG_WRITE(KERROR, "Creating network camera failed. device_ip: {}, port: {}, exception type: {}, what: {}",
									 device_ip, port, typeid(e).name(), e.getMessage());
					continue;
				}
				KERBAL_LOG_WRITE(KINFO, "Creating network camera success. device_ip: {}, port: {}", device_ip, port);

				config_camera(device, v, "network", device_ip);
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
