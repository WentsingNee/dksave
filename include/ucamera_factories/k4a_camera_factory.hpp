/**
 * @file       k4a_camera_factory.hpp
 * @brief
 * @date       2023-09-24
 * @author     Peter
 * @copyright
 *      Peter of [ThinkSpirit Laboratory](http://thinkspirit.org/)
 *   of [Nanjing University of Information Science & Technology](http://www.nuist.edu.cn/)
 *   all rights reserved
 */

#ifndef DKSAVE_K4A_CAMERA_FACTORY_HPP
#define DKSAVE_K4A_CAMERA_FACTORY_HPP

#include "ucamera_factory.hpp"
#include "ucameras/k4a_camera.hpp"
#include "config/k4a_config.hpp"
#include "logger.hpp"

#include <string>
#include <set>
#include <memory>
#include <vector>

#include <cstdlib>

#include <yaml-cpp/node/node.h>


class k4a_camera_factory : public ucamera_factory {

	public:
		virtual
		std::vector<std::unique_ptr<ucamera> >
		find_cameras(YAML::Node &yaml_config) override {
			// 找到并打开 Azure Kinect 设备
			uint32_t device_count = k4a::device::get_installed_count(); // 发现已连接的设备数
			if (0 == device_count) {
				KERBAL_LOG_WRITE(KFATAL, "No k4a camera found.");
				return {};
			}

			KERBAL_LOG_WRITE(KINFO, "Found {} connected cameras.", device_count);

			YAML::Node cameras_node = yaml_config["k4a_cameras"];
			std::vector<std::unique_ptr<ucamera> > cameras;
			std::set<std::string> device_name_used;
			cameras.reserve(device_count);

			for (uint32_t i = 0; i < device_count; ++i) {

				// 打开设备
				try {
					k4a::device device = k4a::device::open(i);
					std::string serial_num = device.get_serialnum();
					KERBAL_LOG_WRITE(KINFO, "Serial num of camara {} is {}", i, serial_num);

					YAML::Node camera_node = cameras_node[serial_num];
					if (!camera_node) {
						KERBAL_LOG_WRITE(KWARNING, "there is no configuration in yaml. serial_num: {}", serial_num);
						continue;
					}

					YAML::Node camera_k4a_config_node = camera_node["config"];
					if (!camera_k4a_config_node) {
						KERBAL_LOG_WRITE(KWARNING, "there is no k4a_config in yaml. serial_num: {}", serial_num);
						continue;
					}

					k4a_device_configuration_t config = yaml_to_k4a_config(camera_k4a_config_node);
					std::string device_name;
					try {
						device_name = camera_node["device_name"].as<std::string>();
					} catch (std::exception const &e) {
						KERBAL_LOG_WRITE(KFATAL, "Parse device_name failed. exception type: {}, what: {}",
										 typeid(e).name(), e.what());
						throw;
					}

					if (device_name_used.find(device_name) != device_name_used.cend()) {
						KERBAL_LOG_WRITE(KFATAL, "device_name: {} has been occupied.", device_name);
						exit(EXIT_FAILURE);
					}
					device_name_used.insert(device_name);

					KERBAL_LOG_WRITE(KINFO,
									 "Open the {}-th camera with arg: serial_num: {}, config: {}, device_name: {}", i,
									 serial_num, config, device_name);
					std::unique_ptr<ucamera> camera = std::make_unique<k4a_camera>(std::move(device), device_name,
																				   std::move(config));
					cameras.push_back(std::move(camera));
				} catch (std::exception const &e) {
					KERBAL_LOG_WRITE(KERROR, "Camara {} open failed. exception type: {}, what: {}", i, typeid(e).name(),
									 e.what());
				} catch (...) {
					KERBAL_LOG_WRITE(KERROR, "Camara {} open failed. what: unknown exception", i);
					continue;
				}
			}

			KERBAL_LOG_WRITE(KINFO, "{} cameras have been opened.", cameras.size());

			return cameras;
		}

};

#endif // DKSAVE_K4A_CAMERA_FACTORY_HPP
