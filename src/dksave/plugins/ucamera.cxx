/**
 * @file       ucamera.cxx
 * @brief
 * @date       2023-09-24
 * @author     Wentsing Nee
 * @copyright
 *      Wentsing Nee of China Agricultural University
 *   all rights reserved
 */

module;

#include <filesystem>
#include <string>

export module dksave.plugins.ucamera;

import dksave.rotate_flag_t;
import dksave.registration_mode_t;
import dksave.working_status;


namespace dksave
{

	export
	class ucamera_base
	{

		protected:
			std::string k_device_name;
			rotate_flag_t k_rotate_flag;
			registration_mode_t k_registration_mode;
			bool k_enable;

		protected:
			ucamera_base(
				std::string const & device_name,
				rotate_flag_t rotate_flag,
				registration_mode_t registration_mode
			) :
				k_device_name(device_name),
				k_rotate_flag(rotate_flag),
				k_registration_mode(registration_mode),
				k_enable(false)
			{
			}

		public:
			std::string const & device_name() const
			{
				return k_device_name;
			}

			dksave::rotate_flag_t
			rotate_flag() const noexcept
			{
				return k_rotate_flag;
			}

			registration_mode_t
			registration_mode() const noexcept
			{
				return k_registration_mode;
			}

			bool enable() const
			{
				return k_enable;
			}
	};


	export
	template <typename Context>
	concept capture_loop_context = requires(
		Context & context,
		std::filesystem::path const & filename_color,
		std::filesystem::path const & filename_depth,
		std::filesystem::path const & filename_pcloud
	)
	{
		{ context.do_capture() };
		{ context.handle_color(filename_color) };
		{ context.handle_depth(filename_depth, filename_pcloud) };
	};

	export
	template <typename Camera>
	concept ucamera = requires(Camera & camera, Camera const & kcamera)
	{
		{ kcamera.device_name() } -> std::convertible_to<std::string>;
		{ kcamera.enable() } -> std::convertible_to<bool>;
		{ camera.start() };
		{ camera.stabilize() };
		{ camera.stop() };
		requires noexcept(camera.stop());
		requires capture_loop_context<typename Camera::capture_loop_context>;
	};

} // namespace dksave
