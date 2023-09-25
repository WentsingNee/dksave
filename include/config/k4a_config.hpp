/**
 * @file       k4a_config.hpp
 * @brief
 * @date       2023-08-26
 * @author     Peter
 * @copyright
 *      Peter of [ThinkSpirit Laboratory](http://thinkspirit.org/)
 *   of [Nanjing University of Information Science & Technology](http://www.nuist.edu.cn/)
 *   all rights reserved
 */

#ifndef DKSAVE_K4A_CONFIG_HPP
#define DKSAVE_K4A_CONFIG_HPP

#include <k4a/k4a.hpp>

#include <fmt/format.h>
#include <yaml-cpp/yaml.h>

#include "logger.hpp"


inline void parse_camera_fps(YAML::Node const & config_yaml, k4a_device_configuration_t & config_k4a)
{
	std::string camera_fps;
	try {
		camera_fps = config_yaml["camera_fps"].as<std::string>();
	} catch (YAML::InvalidNode const & e) {
		KERBAL_LOG_WRITE(KFATAL, "Parse camera_fps error. what: {}", e.what());
		throw;
	}
	if (camera_fps == "5") {
		config_k4a.camera_fps = K4A_FRAMES_PER_SECOND_5;
	} else if (camera_fps == "15") {
		config_k4a.camera_fps = K4A_FRAMES_PER_SECOND_15;
	} else if (camera_fps == "30") {
		config_k4a.camera_fps = K4A_FRAMES_PER_SECOND_30;
	} else {
		throw std::runtime_error(fmt::format("Unknown camera_fps. Got: \"{}\"", camera_fps));
	}
}


inline void parse_color_format(YAML::Node const & config_yaml, k4a_device_configuration_t & config_k4a)
{
	std::string color_format;
	try {
		color_format = config_yaml["color_format"].as<std::string>();
	} catch (YAML::InvalidNode const & e) {
		KERBAL_LOG_WRITE(KFATAL, "Parse camera_fps error. what: {}", e.what());
		throw;
	}
	if (color_format == "BGRA32") {
		config_k4a.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
	} else {
		throw std::runtime_error(fmt::format("Unknown color_format. Got: \"{}\"", color_format));
	}
}

inline void parse_color_resolution(YAML::Node const & config_yaml, k4a_device_configuration_t & config_k4a)
{
	std::string color_resolution;
	try {
		color_resolution = config_yaml["color_resolution"].as<std::string>();
	} catch (YAML::InvalidNode const & e) {
		KERBAL_LOG_WRITE(KFATAL, "Parse camera_fps error. what: {}", e.what());
		throw;
	}
	if (color_resolution == "OFF") {
		config_k4a.color_resolution = K4A_COLOR_RESOLUTION_OFF;
	} else if (color_resolution == "720P") {
		config_k4a.color_resolution = K4A_COLOR_RESOLUTION_720P;
	} else if (color_resolution == "1080P") {
		config_k4a.color_resolution = K4A_COLOR_RESOLUTION_1080P;
	} else if (color_resolution == "1440P") {
		config_k4a.color_resolution = K4A_COLOR_RESOLUTION_1440P;
	} else if (color_resolution == "1536P") {
		config_k4a.color_resolution = K4A_COLOR_RESOLUTION_1536P;
	} else if (color_resolution == "2160P") {
		config_k4a.color_resolution = K4A_COLOR_RESOLUTION_2160P;
	} else if (color_resolution == "3072P") {
		config_k4a.color_resolution = K4A_COLOR_RESOLUTION_3072P;
	} else {
		throw std::runtime_error(fmt::format("Unknown color_resolution. Got: \"{}\"", color_resolution));
	}
}


inline void parse_depth_mode(YAML::Node const & config_yaml, k4a_device_configuration_t & config_k4a)
{
	std::string depth_mode;
	try {
		depth_mode = config_yaml["depth_mode"].as<std::string>();
	} catch (YAML::InvalidNode const & e) {
		KERBAL_LOG_WRITE(KFATAL, "Parse camera_fps error. what: {}", e.what());
		throw;
	}
	if (depth_mode == "OFF") {
		config_k4a.depth_mode = K4A_DEPTH_MODE_OFF;
	} else if (depth_mode == "NFOV_2X2BINNED") {
		config_k4a.depth_mode = K4A_DEPTH_MODE_NFOV_2X2BINNED;
	} else if (depth_mode == "NFOV_UNBINNED") {
		config_k4a.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
	} else if (depth_mode == "WFOV_2X2BINNED") {
		config_k4a.depth_mode = K4A_DEPTH_MODE_WFOV_2X2BINNED;
	} else if (depth_mode == "WFOV_UNBINNED") {
		config_k4a.depth_mode = K4A_DEPTH_MODE_WFOV_UNBINNED;
	} else if (depth_mode == "PASSIVE_IR") {
		config_k4a.depth_mode = K4A_DEPTH_MODE_PASSIVE_IR;
	} else {
		throw std::runtime_error(fmt::format("Unknown depth_mode. Got: \"{}\"", depth_mode));
	}
}


inline k4a_device_configuration_t yaml_to_k4a_config(YAML::Node const & config_yaml)
{
	k4a_device_configuration_t config_k4a = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;

	parse_camera_fps(config_yaml, config_k4a);
	parse_color_format(config_yaml, config_k4a);
	parse_color_resolution(config_yaml, config_k4a);
	parse_depth_mode(config_yaml, config_k4a);

	config_k4a.synchronized_images_only = true; // ensures that depth and color images are both available in the capture

	return config_k4a;
}

template<>
class fmt::formatter<k4a_device_configuration_t>
{

		static
		char const * camera_fps_to_str(k4a_device_configuration_t const & config)
		{
			switch (config.camera_fps) {
				case K4A_FRAMES_PER_SECOND_5: {
					return "5";
				}
				case K4A_FRAMES_PER_SECOND_15: {
					return "15";
				}
				case K4A_FRAMES_PER_SECOND_30: {
					return "30";
				}
				default: {
					return "unknown";
				}
			}
		}

		static
		char const * color_format_to_str(k4a_device_configuration_t const & config)
		{
			switch (config.color_format) {
				case K4A_IMAGE_FORMAT_COLOR_BGRA32: {
					return "BGRA32";
				}
				default: {
					return "unknown";
				}
			}
		}

		static
		char const * color_resolution_to_str(k4a_device_configuration_t const & config)
		{
			switch (config.color_resolution) {
				case K4A_COLOR_RESOLUTION_OFF: {
					return "OFF";
				}
				case K4A_COLOR_RESOLUTION_720P: {
					return "720P";
				}
				case K4A_COLOR_RESOLUTION_1080P: {
					return "1080P";
				}
				case K4A_COLOR_RESOLUTION_1440P: {
					return "1440P";
				}
				case K4A_COLOR_RESOLUTION_1536P: {
					return "1536P";
				}
				case K4A_COLOR_RESOLUTION_2160P: {
					return "2160P";
				}
				case K4A_COLOR_RESOLUTION_3072P: {
					return "3072P";
				}
				default: {
					return "unknown";
				}
			}
		}

		static
		char const * depth_mode_to_str(k4a_device_configuration_t const & config)
		{
			switch (config.depth_mode) {
				case K4A_DEPTH_MODE_OFF: {
					return "OFF";
				}
				case K4A_DEPTH_MODE_NFOV_2X2BINNED: {
					return "NFOV_2X2BINNED";
				}
				case K4A_DEPTH_MODE_NFOV_UNBINNED: {
					return "NFOV_UNBINNED";
				}
				case K4A_DEPTH_MODE_WFOV_2X2BINNED: {
					return "WFOV_2X2BINNED";
				}
				case K4A_DEPTH_MODE_WFOV_UNBINNED: {
					return "WFOV_UNBINNED";
				}
				case K4A_DEPTH_MODE_PASSIVE_IR: {
					return "PASSIVE_IR";
				}
				default: {
					return "unknown";
				}
			}
		}

	public:

		auto format(k4a_device_configuration_t const & config, format_context & ctx) const -> format_context::iterator
		{

			fmt::format_to(
					ctx.out(),
					"{{\n"
					"camera_fps: {}\n"
					"color_format: {}\n"
					"color_resolution: {}\n"
					"depth_mode: {}\n"
					"}}",
					camera_fps_to_str(config),
					color_format_to_str(config),
					color_resolution_to_str(config),
					depth_mode_to_str(config)
			);

			return ctx.out();
		}

		constexpr auto parse(format_parse_context & ctx) -> format_parse_context::iterator
		{
			return ctx.begin();
		}
};


#endif // DKSAVE_K4A_CONFIG_HPP
