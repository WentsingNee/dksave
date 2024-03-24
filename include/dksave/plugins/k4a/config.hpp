/**
 * @file       config.hpp
 * @brief
 * @date       2023-08-26
 * @author     Wentsing Nee
 * @copyright
 *      Wentsing Nee of China Agricultural University
 *   all rights reserved
 */

#ifndef DKSAVE_PLUGINS_K4A_CONFIG_HPP
#define DKSAVE_PLUGINS_K4A_CONFIG_HPP

#include <stdexcept>

#include <cstring>

#include <fmt/format.h>
#include <k4a/k4a.hpp>


namespace dksave::plugins_k4a
{

	inline
	k4a_fps_t str_to_camera_fps(char const * s)
	{
		if (strcmp(s, "5") == 0) {
			return K4A_FRAMES_PER_SECOND_5;
		}
		if (strcmp(s, "15") == 0) {
			return K4A_FRAMES_PER_SECOND_15;
		}
		if (strcmp(s, "30") == 0) {
			return K4A_FRAMES_PER_SECOND_30;
		}
		throw std::runtime_error(fmt::format("Unknown camera_fps. Got: \"{}\"", s));
	}

	inline
	k4a_image_format_t str_to_color_format(char const * s)
	{
		if (strcmp(s, "BGRA32") == 0) {
			return K4A_IMAGE_FORMAT_COLOR_BGRA32;
		}
		throw std::runtime_error(fmt::format("Unknown color_format. Got: \"{}\"", s));
	}

	inline
	k4a_color_resolution_t str_to_color_resolution(char const * s)
	{
		if (strcmp(s, "OFF") == 0) {
			return K4A_COLOR_RESOLUTION_OFF;
		}
		if (strcmp(s, "720P") == 0) {
			return K4A_COLOR_RESOLUTION_720P;
		}
		if (strcmp(s, "1080P") == 0) {
			return K4A_COLOR_RESOLUTION_1080P;
		}
		if (strcmp(s, "1440P") == 0) {
			return K4A_COLOR_RESOLUTION_1440P;
		}
		if (strcmp(s, "1536P") == 0) {
			return K4A_COLOR_RESOLUTION_1536P;
		}
		if (strcmp(s, "2160P") == 0) {
			return K4A_COLOR_RESOLUTION_2160P;
		}
		if (strcmp(s, "3072P") == 0) {
			return K4A_COLOR_RESOLUTION_3072P;
		}
		throw std::runtime_error(fmt::format("Unknown color_resolution. Got: \"{}\"", s));
	}

	inline
	k4a_depth_mode_t str_to_depth_mode(char const * s)
	{
		if (strcmp(s, "OFF") == 0) {
			return K4A_DEPTH_MODE_OFF;
		}
		if (strcmp(s, "NFOV_2X2BINNED") == 0) {
			return K4A_DEPTH_MODE_NFOV_2X2BINNED;
		}
		if (strcmp(s, "NFOV_UNBINNED") == 0) {
			return K4A_DEPTH_MODE_NFOV_UNBINNED;
		}
		if (strcmp(s, "WFOV_2X2BINNED") == 0) {
			return K4A_DEPTH_MODE_WFOV_2X2BINNED;
		}
		if (strcmp(s, "WFOV_UNBINNED") == 0) {
			return K4A_DEPTH_MODE_WFOV_UNBINNED;
		}
		if (strcmp(s, "PASSIVE_IR") == 0) {
			return K4A_DEPTH_MODE_PASSIVE_IR;
		}
		throw std::runtime_error(fmt::format("Unknown depth_mode. Got: \"{}\"", s));
	}

} // namespace dksave::plugins_k4a


template <>
class fmt::formatter<k4a_device_configuration_t>
{

		static
		char const * camera_fps_to_str(k4a_device_configuration_t const & config)
		{
			switch (config.camera_fps) {
				case K4A_FRAMES_PER_SECOND_5: return "5";
				case K4A_FRAMES_PER_SECOND_15: return "15";
				case K4A_FRAMES_PER_SECOND_30: return "30";
				default: return "unknown";
			}
		}

		static
		char const * color_format_to_str(k4a_device_configuration_t const & config)
		{
			switch (config.color_format) {
				case K4A_IMAGE_FORMAT_COLOR_MJPG: return "MJPG";
				case K4A_IMAGE_FORMAT_COLOR_NV12: return "NV12";
				case K4A_IMAGE_FORMAT_COLOR_YUY2: return "YUY2";
				case K4A_IMAGE_FORMAT_COLOR_BGRA32: return "BGRA32";
				case K4A_IMAGE_FORMAT_DEPTH16: return "DEPTH16";
				case K4A_IMAGE_FORMAT_IR16: return "IR16";
				case K4A_IMAGE_FORMAT_CUSTOM8: return "CUSTOM8";
				case K4A_IMAGE_FORMAT_CUSTOM16: return "CUSTOM16";
				case K4A_IMAGE_FORMAT_CUSTOM: return "Custom";
				default: return "unknown";
			}
		}

		static
		char const * color_resolution_to_str(k4a_device_configuration_t const & config)
		{
			switch (config.color_resolution) {
				case K4A_COLOR_RESOLUTION_OFF: return "OFF";
				case K4A_COLOR_RESOLUTION_720P: return "720P";
				case K4A_COLOR_RESOLUTION_1080P: return "1080P";
				case K4A_COLOR_RESOLUTION_1440P: return "1440P";
				case K4A_COLOR_RESOLUTION_1536P: return "1536P";
				case K4A_COLOR_RESOLUTION_2160P: return "2160P";
				case K4A_COLOR_RESOLUTION_3072P: return "3072P";
				default: return "unknown";
			}
		}

		static
		char const * depth_mode_to_str(k4a_device_configuration_t const & config)
		{
			switch (config.depth_mode) {
				case K4A_DEPTH_MODE_OFF: return "OFF";
				case K4A_DEPTH_MODE_NFOV_2X2BINNED: return "NFOV_2X2BINNED";
				case K4A_DEPTH_MODE_NFOV_UNBINNED: return "NFOV_UNBINNED";
				case K4A_DEPTH_MODE_WFOV_2X2BINNED: return "WFOV_2X2BINNED";
				case K4A_DEPTH_MODE_WFOV_UNBINNED: return "WFOV_UNBINNED";
				case K4A_DEPTH_MODE_PASSIVE_IR: return "PASSIVE_IR";
				default: return "unknown";
			}
		}

	public:

		auto
		format(k4a_device_configuration_t const & config, format_context & ctx) const -> format_context::iterator
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

		constexpr
		auto parse(format_parse_context & ctx) -> format_parse_context::iterator
		{
			return ctx.begin();
		}
};

#endif // DKSAVE_PLUGINS_K4A_CONFIG_HPP
