/**
 * @file       registration_mode_t.hpp
 * @brief
 * @date       2024-09-12
 * @author     Wentsing Nee
 * @copyright
 *      Wentsing Nee of China Agricultural University
 *   all rights reserved
 */

#ifndef DKSAVE_REGISTRATION_MODE_T_HPP
#define DKSAVE_REGISTRATION_MODE_T_HPP

#include <stdexcept>
#include <cstring>

#include <fmt/format.h>


namespace dksave
{

	/**
	 * 配准模式
	 */
	enum class registration_mode_t
	{
			DEPTH_TO_COLOR, ///< 深度向可见光配准
			COLOR_TO_DEPTH, ///< 可见光向深度配准
			KEEP, ///< 各自保持不变
	};

	inline
	registration_mode_t str_to_registration_mode(char const * s)
	{
		if (strcmp(s, "DEPTH_TO_COLOR") == 0) {
			return registration_mode_t::DEPTH_TO_COLOR;
		}
		if (strcmp(s, "COLOR_TO_DEPTH") == 0) {
			return registration_mode_t::COLOR_TO_DEPTH;
		}
		if (strcmp(s, "KEEP") == 0) {
			return registration_mode_t::KEEP;
		}
		throw std::runtime_error(fmt::format("Unknown registration_mode. Got: \"{}\"", s));
	}

} // namespace dksave


template <>
class fmt::formatter<dksave::registration_mode_t>
{

		static
		char const *
		registration_mode_t_to_str(dksave::registration_mode_t const & registration_mode)
		{
			switch (registration_mode) {
				case dksave::registration_mode_t::DEPTH_TO_COLOR: return "DEPTH_TO_COLOR";
				case dksave::registration_mode_t::COLOR_TO_DEPTH: return "COLOR_TO_DEPTH";
				case dksave::registration_mode_t::KEEP: return "KEEP";
			}
			return "unknown";
		}


	public:

		auto
		format(
			dksave::registration_mode_t const & registration_mode,
			format_context & ctx
		) const
		{
			fmt::format_to(
				ctx.out(),
				"{}",
				registration_mode_t_to_str(registration_mode)
			);
			return ctx.out();
		}

		constexpr
		auto parse(format_parse_context & ctx)
		{
			return ctx.begin();
		}
};

#endif // DKSAVE_REGISTRATION_MODE_T_HPP
