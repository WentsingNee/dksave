/**
 * @file       ucamera.hpp
 * @brief
 * @date       2023-09-24
 * @author     Wentsing Nee
 * @copyright
 *      Wentsing Nee of China Agricultural University
 *   all rights reserved
 */

#ifndef DKSAVE_PLUGINS_UCAMERA_HPP
#define DKSAVE_PLUGINS_UCAMERA_HPP

#include "dksave/working_status.hpp"

#include <string>


class ucamera_base {

	protected:
		std::string k_device_name;
		bool k_enable;
		working_status previous_status = working_status::WORK;

	protected:
		ucamera_base(std::string const &device_name) :
				k_device_name(device_name),
				k_enable(false) {
		}

	public:
		std::string const &device_name() const {
			return k_device_name;
		}

		bool enable() const {
			return k_enable;
		}
};


template<typename Context>
concept capture_loop_context = requires(Context & context, std::string const &date, std::string const &timestamp) {
	{ context.do_capture() };
	{ context.handle_color(date, timestamp) };
	{ context.handle_depth(date, timestamp) };
};

template<typename Camera>
concept ucamera = requires(Camera & camera, Camera const & kcamera)
{
	{ camera.previous_status } -> std::same_as<working_status &>;
	{ kcamera.device_name() } -> std::convertible_to<std::string>;
	{ kcamera.enable() } -> std::convertible_to<bool>;
	{ camera.start() };
	{ camera.stabilize() };
	{ camera.stop() };
	requires noexcept(camera.stop());
	requires capture_loop_context<typename Camera::capture_loop_context>;
};

#endif // DKSAVE_PLUGINS_UCAMERA_HPP
