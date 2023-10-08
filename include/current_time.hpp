/**
 * @file       current_time.hpp
 * @brief
 * @date       2022-10-06
 * @author     Peter
 * @copyright
 *      Peter of [ThinkSpirit Laboratory](http://thinkspirit.org/)
 *   of [Nanjing University of Information Science & Technology](http://www.nuist.edu.cn/)
 *   all rights reserved
 */

#ifndef DKSAVE_CURRENT_TIME_HPP
#define DKSAVE_CURRENT_TIME_HPP

#include <string>
#include <chrono>

#include <ctime>
#include <windows.h>

#include <fmt/format.h>

inline
std::string format_systime_to_date(const SYSTEMTIME & time)
{
	return fmt::format("{:4}-{:02}-{:02}",
		time.wYear, time.wMonth, time.wDay
	);
}

inline
std::string format_systime_to_time(const SYSTEMTIME & time)
{
	return fmt::format("{:4}-{:02}-{:02} {:02}:{:02}:{:02}",
		time.wYear, time.wMonth, time.wDay,
		time.wHour, time.wMinute, time.wSecond
	);
}

inline
std::string format_systime_to_timestamp(const SYSTEMTIME & time)
{
	return fmt::format("{:4}-{:02}-{:02}_{:02}-{:02}-{:02}-{:03}",
		time.wYear, time.wMonth, time.wDay,
		time.wHour, time.wMinute, time.wSecond, time.wMilliseconds
   );
}


//template <typename Clock, typename Dur>
//std::string format_time_point_to_time(std::chrono::time_point<Clock, Dur> const & time_point)
//template <typename Clock, typename Dur>
std::string format_time_point_to_time(std::chrono::time_point<std::chrono::system_clock> const & time_point_)
{
	auto time_point = std::chrono::zoned_time<std::chrono::milliseconds>().get_local_time();
	using time_point_t = decltype(time_point);
	auto midnight = std::chrono::floor<std::chrono::days>(time_point);
	auto hour = std::chrono::floor<std::chrono::hours>(time_point);
	auto min = std::chrono::floor<std::chrono::minutes>(time_point);
	auto sec = std::chrono::floor<std::chrono::seconds>(time_point);
	return fmt::format("{:4}-{:02}-{:02} {:02}:{:02}:{:02}",
					   2022, 9, 9,
					   (hour - midnight).count(), (min - hour).count(), (sec - min).count()
	);
}

#endif // DKSAVE_CURRENT_TIME_HPP
