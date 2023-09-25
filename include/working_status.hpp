/**
 * @file       working_status.hpp
 * @brief
 * @date       2023-09-24
 * @author     Peter
 * @copyright
 *      Peter of [ThinkSpirit Laboratory](http://thinkspirit.org/)
 *   of [Nanjing University of Information Science & Technology](http://www.nuist.edu.cn/)
 *   all rights reserved
 */


#ifndef DKSAVE_WORKING_STATUS_HPP
#define DKSAVE_WORKING_STATUS_HPP

#include <chrono>

#include <fmt/format.h>


enum class working_status
{
	SLEEP,
	READY,
	WORK,
};

template<>
class fmt::formatter<working_status>
{
		static char const * to_str(working_status status)
		{
			switch (status) {
				case working_status::SLEEP: {
					return "SLEEP";
				}
				case working_status::READY: {
					return "READY";
				}
				case working_status::WORK: {
					return "WORK";
				}
				default: {
					return "UNKNOWN";
				}
			}
		}

	public:

		auto format(working_status status, format_context & ctx) const -> format_context::iterator
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

using namespace std::chrono_literals;

inline std::chrono::seconds start_time;
inline std::chrono::seconds end_time;
inline auto prepare_time = 1min;


static working_status get_working_status(std::chrono::time_point<std::chrono::system_clock> now)
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


#endif //DKSAVE_WORKING_STATUS_HPP
