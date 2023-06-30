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

#endif // DKSAVE_CURRENT_TIME_HPP
