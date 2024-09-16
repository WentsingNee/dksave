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
#include "dksave/registration_mode_t.hpp"
#include "dksave/rotate_flag_t.hpp"

#include <string>
#include <iostream>
#include <variant>

#include <cstdlib>

#include <k4a/k4a.hpp>
#include <fmt/format.h>
#include <fmt/ranges.h>
#include <yaml-cpp/yaml.h>

#include <kerbal/algorithm/querier/find_if.hpp>
#include <kerbal/container/avl_map.hpp>
#include <kerbal/container/vector.hpp>


namespace dksave::plugins_k4a
{

	class camera_factory
	{

		private:
			static k4a_device_configuration_t parse_yaml_to_config(YAML::Node const & config_yaml)
			{
				k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;

				auto parse_field = [](YAML::Node const & config_yaml, k4a_device_configuration_t & config_k4a, const char * field_name, auto field, auto deserializer) {
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

				if (config.color_resolution != K4A_COLOR_RESOLUTION_OFF && config.depth_mode != K4A_DEPTH_MODE_OFF) {
					config.synchronized_images_only = true; // ensures that depth and color images are both available in the capture
				}

				return config;
			}

			static
			rotate_flag_t parse_yaml_to_rotate_flag(YAML::Node const & rotate_yaml)
			{
				if (!rotate_yaml.IsDefined()) {
					rotate_flag_t default_ = rotate_flag_t::CLOCKWISE_0;
					KERBAL_LOG_WRITE(
						KWARNING, "There is no `rotate` property in yaml, set default to: {}.",
						default_
					);
					return default_;
				}
				std::string s;
				try {
					s = rotate_yaml.as<std::string>();
				} catch (YAML::InvalidNode const & e) {
					KERBAL_LOG_WRITE(KERROR, "Invalid node. what: {}", e.what());
					throw;
				}
				if (s == "CLOCKWISE_0") {
					return rotate_flag_t::CLOCKWISE_0;
				}
				if (s == "CLOCKWISE_90") {
					return rotate_flag_t::CLOCKWISE_90;
				}
				if (s == "CLOCKWISE_180") {
					return rotate_flag_t::CLOCKWISE_180;
				}
				if (s == "CLOCKWISE_270") {
					return rotate_flag_t::CLOCKWISE_270;
				}
				std::string errmsg("Invalid `rotate` property, must be one of: [CLOCKWISE_0, CLOCKWISE_0, CLOCKWISE_0, CLOCKWISE_0]");
				KERBAL_LOG_WRITE(KERROR, "{}", errmsg);
				throw std::invalid_argument(errmsg);
			}

		public:
			static
			void read_sync_groups(
				YAML::Node const & k4a_node,
				kerbal::container::avl_map<std::string, kerbal::container::vector<std::string> > & master_2_sub,
				kerbal::container::avl_map<std::string, std::string> & sub_2_master
			)
			{
				YAML::Node sync_groups_node = k4a_node["sync_groups"];
				if (not sync_groups_node.IsMap()) {
					KERBAL_LOG_WRITE(KFATAL, "k4a.sync_groups node type error");
					exit(EXIT_FAILURE);
				}
				for (auto const & sync_group : sync_groups_node) {
					std::string master_snid = sync_group.first.as<std::string>();
					auto const & subordinates_node = sync_group.second;
					if (not subordinates_node.IsSequence()) {
						KERBAL_LOG_WRITE(KFATAL, "subordinates should be list. master: {}", master_snid);
						exit(EXIT_FAILURE);
					}
					if (auto it = sub_2_master.find(master_snid); it != sub_2_master.cend()) {
						KERBAL_LOG_WRITE(KFATAL, "{} can't be a master because it has been designated as {}'s subordinate", master_snid, it->value());
						exit(EXIT_FAILURE);
					}
					auto & subs_vec = master_2_sub[master_snid];
					for (auto const & subordinate_entry : subordinates_node) {
						std::string subordinate_snid = subordinate_entry.as<std::string>();
						if (master_2_sub.contains(subordinate_snid)) {
							KERBAL_LOG_WRITE(KFATAL, "{} can't be designated as {}'s subordinate because it's a master", subordinate_snid, master_snid);
							exit(EXIT_FAILURE);
						}
						if (auto it = sub_2_master.find(subordinate_snid); it != sub_2_master.cend()) {
							KERBAL_LOG_WRITE(KFATAL, "{} can't be designated as {}'s subordinate because it has been designated as {}'s subordinate before", subordinate_snid,
											 master_snid, it->value());
							exit(EXIT_FAILURE);
						}
						subs_vec.push_back(subordinate_snid);
						sub_2_master.emplace(subordinate_snid, master_snid);
						KERBAL_LOG_WRITE(KINFO, "Add sync relationship. master: {}, subordinate: {}", master_snid, subordinate_snid);
					}
				}
				std::string sync_groups_describe;
				for (auto const & [master_snid, subs_vec] : master_2_sub) {
					sync_groups_describe += fmt::format(
						"master: {}, subordinates: [{}]\n",
						master_snid, fmt::join(subs_vec, ", ")
					);
				}
				KERBAL_LOG_WRITE(KINFO, "sync groups:\n{}", sync_groups_describe);
			}

			static
			kerbal::container::vector<
				std::variant<
					camera,
					kerbal::container::vector<camera>
				>
			>
			find_cameras(YAML::Node const & yaml_config)
			{
				// 找到并打开 Azure Kinect 设备
				uint32_t device_count = k4a::device::get_installed_count(); // 发现已连接的设备数
				if (0 == device_count) {
					KERBAL_LOG_WRITE(KWARNING, "There is no k4a camera found.");
					return {};
				}

				KERBAL_LOG_WRITE(KINFO, "Found {} connected cameras.", device_count);

				YAML::Node k4a_node = yaml_config["k4a"];

				kerbal::container::avl_map<
					std::string,
					kerbal::container::vector<std::string>
				> master_2_sub;
				kerbal::container::avl_map<std::string, std::string> sub_2_master;
				read_sync_groups(k4a_node, master_2_sub, sub_2_master);


				YAML::Node cameras_node = k4a_node["cameras"];
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
						if (master_2_sub.contains(serial_num)) {
							config.wired_sync_mode = K4A_WIRED_SYNC_MODE_MASTER;
						} else if (sub_2_master.contains(serial_num)) {
							config.wired_sync_mode = K4A_WIRED_SYNC_MODE_SUBORDINATE;
						} else {
							config.wired_sync_mode = K4A_WIRED_SYNC_MODE_STANDALONE;
						}

						std::string device_name;
						try {
							device_name = camera_node["device_name"].as<std::string>();
						} catch (std::exception const & e) {
							KERBAL_LOG_WRITE(KFATAL, "Parse device_name failed. exception type: {}, what: {}",
											 typeid(e).name(), e.what());
							exit(EXIT_FAILURE);
						}

						auto uir = dksave::global_settings::add_device_name_occupied(device_name);
						if (!uir.insert_happen()) {
							KERBAL_LOG_WRITE(KFATAL, "device_name: {} has been occupied.", device_name);
							exit(EXIT_FAILURE);
						}

						if (config.color_resolution == K4A_COLOR_RESOLUTION_OFF && config.depth_mode == K4A_DEPTH_MODE_OFF) {
							KERBAL_LOG_WRITE(
								KFATAL, "Invalid config because both color frame and depth frame are disabled. serial_num: {}, device_name: {}",
								serial_num, device_name
							);
							exit(EXIT_FAILURE);
						}

						dksave::rotate_flag_t rotate_flag = parse_yaml_to_rotate_flag(camera_node["rotate"]);

						YAML::Node registration_mode_node = camera_node["registration_mode"];
						registration_mode_t registration_mode = registration_mode_t::DEPTH_TO_COLOR;
						if (config.color_resolution == K4A_COLOR_RESOLUTION_OFF || config.depth_mode == K4A_DEPTH_MODE_OFF) {
							registration_mode = registration_mode_t::KEEP;
						}
						if (!registration_mode_node.IsDefined()) {
							KERBAL_LOG_WRITE(
								KWARNING, "There is no registration_mode in yaml, set default to: {}. serial_num: {}, device_name: {}",
								registration_mode,
								serial_num, device_name
							);
						} else {
							try {
								registration_mode = str_to_registration_mode(registration_mode_node.as<std::string>().c_str());
							} catch (std::exception const & e) {
								KERBAL_LOG_WRITE(
									KFATAL, "Parsing registration_mode failed. serial_num: {}, device_name: {}, exception_type: {}, what: {}",
									serial_num, device_name,
									typeid(e).name(), e.what()
								);
								exit(EXIT_FAILURE);
							} catch (...) {
								KERBAL_LOG_WRITE(
									KFATAL, "Parsing registration_mode failed. serial_num: {}, device_name: {}, exception_type: unknown",
									serial_num, device_name
								);
								exit(EXIT_FAILURE);
							}
							KERBAL_LOG_WRITE(
								KINFO, "Set registration_mode to: {}. serial_num: {}, device_name: {}",
								registration_mode,
								serial_num, device_name
							);
						}

						if (config.color_resolution == K4A_COLOR_RESOLUTION_OFF) {
							if (registration_mode == registration_mode_t::DEPTH_TO_COLOR) {
								KERBAL_LOG_WRITE(
									KFATAL, "Invalid registration_mode because color frame is disabled. serial_num: {}, device_name: {}, registration_mode: {}",
									serial_num, device_name,
									registration_mode
								);
								exit(EXIT_FAILURE);
							}
						}

						if (config.depth_mode == K4A_DEPTH_MODE_OFF) {
							if (registration_mode == registration_mode_t::COLOR_TO_DEPTH) {
								KERBAL_LOG_WRITE(
									KFATAL, "Invalid registration_mode because depth frame is disabled. serial_num: {}, device_name: {}, registration_mode: {}",
									serial_num, device_name,
									registration_mode
								);
								exit(EXIT_FAILURE);
							}
						}

						KERBAL_LOG_WRITE(KINFO,
										 "Open the {}-th camera with arg: serial_num: {}, config: {}, device_name: {}",
										 i,
										 serial_num, config, device_name);
						cameras.emplace_back(
							std::move(device),
							device_name,
							rotate_flag,
							registration_mode,
							config
						);
					} catch (std::exception const & e) {
						KERBAL_LOG_WRITE(KERROR, "Camara {} open failed. exception type: {}, what: {}", i,
										 typeid(e).name(),
										 e.what());
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

				KERBAL_LOG_WRITE(KINFO, "Grouping the master or standalone cameras");

				kerbal::container::vector<
					std::variant<
						camera,
						kerbal::container::vector<camera>
					>
				> cameras_by_group;

				for (auto & camera_ : cameras) {
					std::string device_name = camera_.device_name(); // camera_ will be invalid after moved
					switch (camera_.config().wired_sync_mode) {
						case K4A_WIRED_SYNC_MODE_STANDALONE: {
							cameras_by_group.emplace_back(std::in_place_type<camera>, std::move(camera_));
							KERBAL_LOG_WRITE(KINFO, "Grouped a standalone camera. device name: {}, group: {}", device_name, cameras_by_group.size());
							break;
						}
						case K4A_WIRED_SYNC_MODE_MASTER: {
							cameras_by_group.emplace_back(std::in_place_type<kerbal::container::vector<camera>>);
							auto & group = cameras_by_group.back();
							std::get<1>(group).emplace_back(std::move(camera_));
							KERBAL_LOG_WRITE(KINFO, "Grouped a master camera. device name: {}, group: {}", device_name, cameras_by_group.size());
							break;
						}
					}
				}

				KERBAL_LOG_WRITE(KINFO, "Grouping the subordinate cameras");

				for (auto & sub_camera : cameras) {
					if (sub_camera.config().wired_sync_mode != K4A_WIRED_SYNC_MODE_SUBORDINATE) {
						continue;
					}
					std::string sub_snid = sub_camera.device().get_serialnum();
					std::string const & master_snid = sub_2_master.at(sub_snid);

					auto it = kerbal::algorithm::find_if(
						cameras_by_group.begin(),
						cameras_by_group.end(),
						[&master_snid](auto & group) {
							if (group.index() == 0) {
								return false;
							}
							return std::get<1>(group).front().device().get_serialnum() == master_snid;
						}
					);

					if (it != cameras_by_group.end()) {
						std::string device_name = sub_camera.device_name(); // sub_camera will be invalid after moved
						std::get<1>(*it).emplace_back(std::move(sub_camera));
						KERBAL_LOG_WRITE(KINFO, "Grouped a subordinate camera. device name: {}, group: {}", device_name, it - cameras_by_group.begin());
					}

				}

				return cameras_by_group;
			}

	};

	static_assert(dksave::ucamera_factory<camera_factory>,
				  "k4a_camera_factory doesn't meet the requirement of ucamera_factory");

} // namespace dksave::plugins_k4a

#endif // DKSAVE_PLUGINS_K4A_CAMERA_FACTORY_HPP
