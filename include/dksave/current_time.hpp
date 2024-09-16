/**
 * @file       current_time.hpp
 * @brief
 * @date       2022-10-06
 * @author     Wentsing Nee
 * @copyright
 *      Wentsing Nee of China Agricultural University
 *   all rights reserved
 */

#ifndef DKSAVE_CURRENT_TIME_HPP
#define DKSAVE_CURRENT_TIME_HPP

#include <string>
#include <chrono>

#include <fmt/chrono.h>
#include <fmt/format.h>


namespace dksave
{

	template <typename Duration>
	std::string
	format_systime_to_date(std::chrono::sys_time<Duration> const & sys_time)
	{
		auto local_time = std::chrono::current_zone()->to_local<Duration>(sys_time);
		auto floor_time = std::chrono::floor<std::chrono::days>(local_time);
		return fmt::format("{:%Y-%m-%d}", floor_time);
	}

	template <typename Duration>
	std::string
	format_systime_to_datetime(std::chrono::sys_time<Duration> const & sys_time)
	{
		auto local_time = std::chrono::current_zone()->to_local<Duration>(sys_time);
		auto floor_time = std::chrono::floor<std::chrono::seconds>(local_time);
		return fmt::format("{:%Y-%m-%d %H:%M:%S}", floor_time);
	}

	template <typename Duration>
	std::string
	format_systime_to_datetime_milli(std::chrono::sys_time<Duration> const & sys_time)
	{
		auto local_time = std::chrono::current_zone()->to_local<Duration>(sys_time);
		auto floor_time = std::chrono::floor<std::chrono::milliseconds>(local_time);
		auto floor_seconds = std::chrono::floor<std::chrono::seconds>(floor_time);
		return fmt::format("{:%Y-%m-%d_%H-%M-%S}-{:03}", floor_seconds, (floor_time - floor_seconds).count());
	}

} // namespace dksave

#endif // DKSAVE_CURRENT_TIME_HPP
