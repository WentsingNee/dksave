/**
 * @file       working_status.hpp
 * @brief
 * @date       2023-09-24
 * @author     Wentsing Nee
 * @copyright
 *      Wentsing Nee of China Agricultural University
 *   all rights reserved
 */

#ifndef DKSAVE_WORKING_STATUS_HPP
#define DKSAVE_WORKING_STATUS_HPP

#include <chrono>

#include <fmt/format.h>


using namespace std::chrono_literals;

inline std::chrono::seconds start_time;
inline std::chrono::seconds end_time;
inline auto prepare_time = 1min;


namespace dksave
{

	enum class working_status
	{
			SLEEP,
			READY,
			WORK,
	};

	static
	working_status get_working_status(std::chrono::time_point<std::chrono::system_clock> now)
	{
		const std::chrono::time_zone * tz = std::chrono::current_zone();
		auto local_now = tz->to_local(now);
		auto today_midnight = std::chrono::floor<std::chrono::days>(local_now);
		auto time_since_midnight = local_now - today_midnight;

		if (start_time <= time_since_midnight && time_since_midnight < end_time) {
			return working_status::WORK;
		}
		auto t = (time_since_midnight + prepare_time) % 24h;
		if (start_time <= t && t < end_time) {
			return working_status::READY;
		}
		return working_status::SLEEP;
	}

} // namespace dksave


template <>
class fmt::formatter<dksave::working_status>
{
		static char const * to_str(dksave::working_status status)
		{
			switch (status) {
				case dksave::working_status::SLEEP: {
					return "SLEEP";
				}
				case dksave::working_status::READY: {
					return "READY";
				}
				case dksave::working_status::WORK: {
					return "WORK";
				}
				default: {
					return "UNKNOWN";
				}
			}
		}

	public:

		auto format(dksave::working_status status, format_context & ctx) const -> format_context::iterator
		{

			fmt::format_to(
				ctx.out(),
				"{}", to_str(status)
			);

			return ctx.out();
		}

		constexpr auto parse(format_parse_context & ctx) -> format_parse_context::iterator
		{
			return ctx.begin();
		}
};


#endif // DKSAVE_WORKING_STATUS_HPP
