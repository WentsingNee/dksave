/**
 * @file       camera_factory.hpp
 * @brief
 * @date       2023-09-24
 * @author     Wentsing Nee
 * @copyright
 *      Wentsing Nee of China Agricultural University
 *   all rights reserved
 */

#ifndef DKSAVE_PLUGINS_K4A_CAMERA_FACTORY_HPP
#define DKSAVE_PLUGINS_K4A_CAMERA_FACTORY_HPP

#include "camera.hpp"
#include "config.hpp"

#include "dksave/plugins/ucamera_factory.hpp"
#include "dksave/global_settings.hpp"
#include "dksave/logger.hpp"

#include <string>

#include <cstdlib>

#include <k4a/k4a.hpp>
#include <yaml-cpp/yaml.h>

#include <kerbal/container/vector.hpp>


namespace dksave::plugins_k4a
{

	class camera_factory
	{

		private:
			static
			k4a_device_configuration_t parse_yaml_to_config(YAML::Node const & config_yaml)
			{
				k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;

				auto parse_field = [](
					YAML::Node const & config_yaml,
					k4a_device_configuration_t & config_k4a,
					const char * field_name,
					auto field,
					auto deserializer
				) {
					std::string s;
					try {
						s = config_yaml[field_name].as<std::string>();
					} catch (YAML::InvalidNode const & e) {
						KERBAL_LOG_WRITE(KERROR, "Invalid node. key: {}, what: {}", field_name, e.what());
						throw;
					}
					config_k4a.*field = deserializer(s.c_str());
				};

				parse_field(config_yaml, config, "camera_fps", &k4a_device_configuration_t::camera_fps, str_to_camera_fps);
				parse_field(config_yaml, config, "color_format", &k4a_device_configuration_t::color_format, str_to_color_format);
				parse_field(config_yaml, config, "color_resolution", &k4a_device_configuration_t::color_resolution, str_to_color_resolution);
				parse_field(config_yaml, config, "depth_mode", &k4a_device_configuration_t::depth_mode, str_to_depth_mode);

				config.synchronized_images_only = true; // ensures that depth and color images are both available in the capture

				return config;
			}

		public:
			static
			kerbal::container::vector<camera>
			find_cameras(YAML::Node const & yaml_config)
			{
				// 找到并打开 Azure Kinect 设备
				uint32_t device_count = k4a::device::get_installed_count(); // 发现已连接的设备数
				if (0 == device_count) {
					KERBAL_LOG_WRITE(KWARNING, "There is no k4a camera found.");
					return {};
				}

				KERBAL_LOG_WRITE(KINFO, "Found {} connected cameras.", device_count);

				YAML::Node cameras_node = yaml_config["k4a_cameras"];
				kerbal::container::vector<camera> cameras;
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

						k4a_device_configuration_t config = parse_yaml_to_config(camera_k4a_config_node);
						std::string device_name;
						try {
							device_name = camera_node["device_name"].as<std::string>();
						} catch (std::exception const & e) {
							KERBAL_LOG_WRITE(
								KFATAL, "Parse device_name failed. exception type: {}, what: {}",
								typeid(e).name(),
								e.what()
							);
							exit(EXIT_FAILURE);
						}

						auto uir = dksave::global_settings::add_device_name_occupied(device_name);
						if (!uir.insert_happen()) {
							KERBAL_LOG_WRITE(KFATAL, "device_name: {} has been occupied.", device_name);
							exit(EXIT_FAILURE);
						}

						KERBAL_LOG_WRITE(
							KINFO,
							"Open the {}-th camera with arg: serial_num: {}, config: {}, device_name: {}",
							i,
							serial_num, config, device_name
						);
						cameras.emplace_back(std::move(device), device_name, config);
					} catch (std::exception const & e) {
						KERBAL_LOG_WRITE(
							KERROR, "Camara {} open failed. exception type: {}, what: {}", i,
							typeid(e).name(), e.what()
						);
					} catch (...) {
						KERBAL_LOG_WRITE(KERROR, "Camara {} open failed. what: unknown exception", i);
						continue;
					}
				}

				if (0 == cameras.size()) {
					KERBAL_LOG_WRITE(KWARNING, "There is no k4a camera opened.");
				} else {
					KERBAL_LOG_WRITE(KINFO, "k4a cameras found finished. found: {}", cameras.size());
				}

				return cameras;
			}

	};

	static_assert(
		dksave::ucamera_factory<camera_factory>,
		"k4a_camera_factory doesn't meet the requirement of ucamera_factory"
	);

} // namespace dksave::plugins_k4a

#endif // DKSAVE_PLUGINS_K4A_CAMERA_FACTORY_HPP
