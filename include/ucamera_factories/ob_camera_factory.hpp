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
#include <set>
#include <map>
#include <memory>
#include <vector>

#include <cstdlib>

#include <libobsensor/ObSensor.hpp>
#include <yaml-cpp/yaml.h>


class ob_camera_factory : public ucamera_factory {

	public:
		virtual
		std::vector<std::unique_ptr<ucamera> >
		find_cameras(YAML::Node &yaml_config) override {
			std::vector<std::unique_ptr<ucamera> > cameras;
			std::set<std::string> device_name_used;

			ob::Context ctx;

			YAML::Node cameras_node = yaml_config["ob_cameras"];
			int i = -1;
			for (auto const & e : cameras_node) {
				++i;
				// 打开设备
				try {
					auto const & k = e.first;
					auto const & v = e.second;

					std::string device_ip = k.as<std::string>();

					KERBAL_LOG_WRITE(KINFO, "Try opening net camera {}", device_ip);
					auto device = ctx.createNetDevice(device_ip.c_str(), 8090);
					KERBAL_LOG_WRITE(KINFO, "Net camera {} open success", device_ip);

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

					if (device_name_used.find(device_name) != device_name_used.cend()) {
						KERBAL_LOG_WRITE(KFATAL, "device_name: {} has been occupied.", device_name);
						exit(EXIT_FAILURE);
					}
					device_name_used.insert(device_name);

					KERBAL_LOG_WRITE(KINFO,
									 "Open the {}-th camera with arg: serial_num: {}, device_name: {}", i,
									 serial_num, device_name);
					cameras.push_back(std::make_unique<ob_camera>(std::move(device), device_name));
				} catch (ob::Error const& e) {
					KERBAL_LOG_WRITE(KERROR, "Camara {} open failed. exception type: {}, what: {}", i, typeid(e).name(),
									 e.getMessage());
				} catch (std::exception const& e) {
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

#endif // DKSAVE_OB_CAMERA_FACTORY_HPP
