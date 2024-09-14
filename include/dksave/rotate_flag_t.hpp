/**
 * @file       rotate_flag_t.hpp
 * @brief
 * @date       2024-09-14
 * @author     Peter
 * @copyright
 *      Wentsing Nee of China Agricultural University
 *   all rights reserved
 */

#ifndef DKSAVE_ROTATE_FLAG_T_HPP
#define DKSAVE_ROTATE_FLAG_T_HPP

#include <stdexcept>
#include <cstring>

#include <fmt/format.h>


namespace dksave
{

	enum class rotate_flag_t
	{
			CLOCKWISE_0,
			CLOCKWISE_90,
			CLOCKWISE_180,
			CLOCKWISE_270,
	};

	inline
	rotate_flag_t str_to_rotate_flag(char const * s)
	{
		if (strcmp(s, "CLOCKWISE_0") == 0) {
			return rotate_flag_t::CLOCKWISE_0;
		}
		if (strcmp(s, "CLOCKWISE_90") == 0) {
			return rotate_flag_t::CLOCKWISE_90;
		}
		if (strcmp(s, "CLOCKWISE_180") == 0) {
			return rotate_flag_t::CLOCKWISE_180;
		}
		if (strcmp(s, "CLOCKWISE_270") == 0) {
			return rotate_flag_t::CLOCKWISE_270;
		}
		throw std::runtime_error(fmt::format("Unknown rotate_flag. Got: \"{}\"", s));
	}

} // namespace dksave


template <>
class fmt::formatter<dksave::rotate_flag_t>
{
		static
		char const *
		rotate_flag_t_to_str(dksave::rotate_flag_t const & rotate_flag)
		{
			switch (rotate_flag) {
				case dksave::rotate_flag_t::CLOCKWISE_0: return "CLOCKWISE_0";
				case dksave::rotate_flag_t::CLOCKWISE_90: return "CLOCKWISE_90";
				case dksave::rotate_flag_t::CLOCKWISE_180: return "CLOCKWISE_180";
				case dksave::rotate_flag_t::CLOCKWISE_270: return "CLOCKWISE_270";
			}
			return "unknown";
		}

	public:
		auto
		format(
			dksave::rotate_flag_t const & rotate_flag,
			format_context & ctx
		) const
		{
			fmt::format_to(
				ctx.out(),
				"{}",
				rotate_flag_t_to_str(rotate_flag)
			);
			return ctx.out();
		}

		constexpr
		auto parse(format_parse_context & ctx)
		{
			return ctx.begin();
		}
};

#endif // DKSAVE_ROTATE_FLAG_T_HPP
