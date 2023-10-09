/**
 * @file       ob_config.hpp
 * @brief
 * @date       2023/10/9
 * @author     Peter
 * @copyright
 *      Peter of [ThinkSpirit Laboratory](http://thinkspirit.org/)
 *   of [Nanjing University of Information Science & Technology](http://www.nuist.edu.cn/)
 *   all rights reserved
 */


#ifndef DKSAVE_OB_CONFIG_HPP
#define DKSAVE_OB_CONFIG_HPP

#include <cstring>

#include <fmt/format.h>

#include <libobsensor/ObSensor.hpp>


OBFormat str_to_OB_FORMAT(char const * s)
{
	if (strcmp(s, "OB_FORMAT_YUYV") == 0)  return OB_FORMAT_YUYV;
	if (strcmp(s, "OB_FORMAT_YUY2") == 0)  return OB_FORMAT_YUY2;
	if (strcmp(s, "OB_FORMAT_UYVY") == 0)  return OB_FORMAT_UYVY;
	if (strcmp(s, "OB_FORMAT_NV12") == 0)  return OB_FORMAT_NV12;
	if (strcmp(s, "OB_FORMAT_NV21") == 0)  return OB_FORMAT_NV21;
	if (strcmp(s, "OB_FORMAT_MJPG") == 0)  return OB_FORMAT_MJPG;
	if (strcmp(s, "OB_FORMAT_H264") == 0)  return OB_FORMAT_H264;
	if (strcmp(s, "OB_FORMAT_H265") == 0)  return OB_FORMAT_H265;
	if (strcmp(s, "OB_FORMAT_Y16") == 0)  return OB_FORMAT_Y16;
	if (strcmp(s, "OB_FORMAT_Y8") == 0)  return OB_FORMAT_Y8;
	if (strcmp(s, "OB_FORMAT_Y10") == 0)  return OB_FORMAT_Y10;
	if (strcmp(s, "OB_FORMAT_Y11") == 0)  return OB_FORMAT_Y11;
	if (strcmp(s, "OB_FORMAT_Y12") == 0)  return OB_FORMAT_Y12;
	if (strcmp(s, "OB_FORMAT_GRAY") == 0)  return OB_FORMAT_GRAY;
	if (strcmp(s, "OB_FORMAT_HEVC") == 0)  return OB_FORMAT_HEVC;
	if (strcmp(s, "OB_FORMAT_I420") == 0)  return OB_FORMAT_I420;
	if (strcmp(s, "OB_FORMAT_ACCEL") == 0)  return OB_FORMAT_ACCEL;
	if (strcmp(s, "OB_FORMAT_GYRO") == 0)  return OB_FORMAT_GYRO;
	if (strcmp(s, "OB_FORMAT_POINT") == 0)  return OB_FORMAT_POINT;
	if (strcmp(s, "OB_FORMAT_RGB_POINT") == 0)  return OB_FORMAT_RGB_POINT;
	if (strcmp(s, "OB_FORMAT_RLE") == 0)  return OB_FORMAT_RLE;
	if (strcmp(s, "OB_FORMAT_RGB") == 0)  return OB_FORMAT_RGB;
	if (strcmp(s, "OB_FORMAT_BGR") == 0)  return OB_FORMAT_BGR;
	if (strcmp(s, "OB_FORMAT_Y14") == 0)  return OB_FORMAT_Y14;
	if (strcmp(s, "OB_FORMAT_BGRA") == 0)  return OB_FORMAT_BGRA;
	if (strcmp(s, "OB_FORMAT_COMPRESSED") == 0)  return OB_FORMAT_COMPRESSED;
	if (strcmp(s, "OB_FORMAT_RVL") == 0)  return OB_FORMAT_RVL;
	if (strcmp(s, "OB_FORMAT_UNKNOWN") == 0)  return OB_FORMAT_UNKNOWN;
	throw std::runtime_error("unknown OB_FORMAT");
}

template<>
struct fmt::formatter<OBFormat> {

		static
		char const * OB_FORMAT_to_str(OBFormat format) {
			switch (format) {
				case OB_FORMAT_YUYV  : return    "YUYV format";
				case OB_FORMAT_YUY2  : return    "YUY2 format (the actual format is the same as YUYV)";
				case OB_FORMAT_UYVY  : return    "UYVY format";
				case OB_FORMAT_NV12  : return    "NV12 format";
				case OB_FORMAT_NV21  : return    "NV21 format";
				case OB_FORMAT_MJPG  : return    "MJPEG encoding format";
				case OB_FORMAT_H264  : return    "H.264 encoding format";
				case OB_FORMAT_H265  : return    "H.265 encoding format";
				case OB_FORMAT_Y16   : return    "Y16 format, single channel 16-bit depth";
				case OB_FORMAT_Y8    : return    "Y8 format, single channel 8-bit depth";
				case OB_FORMAT_Y10   : return    "Y10 format, single channel 10-bit depth (SDK will unpack into Y16 by default)";
				case OB_FORMAT_Y11   : return    "Y11 format, single channel 11-bit depth (SDK will unpack into Y16 by default)";
				case OB_FORMAT_Y12   : return    "Y12 format, single channel 12-bit depth (SDK will unpack into Y16 by default)";
				case OB_FORMAT_GRAY  : return    "GRAY (the actual format is the same as YUYV)";
				case OB_FORMAT_HEVC  : return    "HEVC encoding format (the actual format is the same as H265)";
				case OB_FORMAT_I420  : return    "I420 format";
				case OB_FORMAT_ACCEL : return    "Acceleration data format";
				case OB_FORMAT_GYRO  : return    "Gyroscope data format";
				case OB_FORMAT_POINT : return    "XYZ 3D coordinate point format";
				case OB_FORMAT_RGB_POINT : return   "XYZ 3D coordinate point format with RGB information";
				case OB_FORMAT_RLE   : return   "RLE pressure test format (SDK will be unpacked into Y16 by default)";
				case OB_FORMAT_RGB   : return   "RGB format (actual RGB888) ";
				case OB_FORMAT_BGR   : return   "BGR format (actual BGR888)";
				case OB_FORMAT_Y14   : return   "Y14 format, single channel 14-bit depth (SDK will unpack into Y16 by default)";
				case OB_FORMAT_BGRA  : return   "BGRA format";
				case OB_FORMAT_COMPRESSED: return   "Compression format";
				case OB_FORMAT_RVL   : return   "RVL pressure test format (SDK will be unpacked into Y16 by default)";
				case OB_FORMAT_UNKNOWN   : return "unknown format";
				default: return "unknown";
			}
		}

		auto
		format(OBFormat const & format, format_context &ctx) const -> format_context::iterator {
			fmt::format_to(
					ctx.out(),
					"{}", OB_FORMAT_to_str(format)
			);
			return ctx.out();
		}

		constexpr auto parse(format_parse_context &ctx) -> format_parse_context::iterator {
			return ctx.begin();
		}
};


template<>
struct fmt::formatter<ob::VideoStreamProfile> {

	auto
	format(ob::VideoStreamProfile const & profile, format_context &ctx) const -> format_context::iterator {
		fmt::format_to(
				ctx.out(),
				"width: {:5}, height: {:5}, format: {}, fps: {}",
				profile.width(), profile.height(),
				profile.format(), profile.fps()
		);
		return ctx.out();
	}

	constexpr auto parse(format_parse_context &ctx) -> format_parse_context::iterator {
		return ctx.begin();
	}

};

#endif // DKSAVE_OB_CONFIG_HPP
