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
std::string current_date()
{
	std::time_t now = std::time(NULL);
	std::tm * localtime = std::localtime(&now);

	return fmt::format("{:4}-{:02}-{:02}",
					   1900 + localtime->tm_year, 1 + localtime->tm_mon, localtime->tm_mday);
}

inline
std::string current_time()
{
	std::time_t now = std::time(NULL);
	std::tm * localtime = std::localtime(&now);

	return fmt::format("{:4}-{:02}-{:02} {:02}:{:02}:{:02}",
					   1900 + localtime->tm_year, 1 +localtime->tm_mon, localtime->tm_mday,
					   localtime->tm_hour, localtime->tm_min, localtime->tm_sec);
}

inline
std::string current_timestamp()
{
	SYSTEMTIME lpsystime1;
	GetLocalTime(&lpsystime1);
	return fmt::format("{:4}-{:2}-{:2}_{:2}-{:2}-{:2}-{:3}",
					   lpsystime1.wYear, lpsystime1.wMonth, lpsystime1.wDay,
					   lpsystime1.wHour, lpsystime1.wMinute, lpsystime1.wSecond, lpsystime1.wMilliseconds
   );
}

#endif // DKSAVE_CURRENT_TIME_HPP
