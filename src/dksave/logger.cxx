/**
 * @file       logger.cxx
 * @brief
 * @date       2022-10-06
 * @author     Wentsing Nee
 * @copyright
 *      Wentsing Nee of China Agricultural University
 *   all rights reserved
 */

module;

#include <kerbal/utility/costream.hpp>

#include <iostream>
#include <mutex>

#include <ctime>

#include <fmt/format.h>

export module dksave.logger;

import dksave.current_time;


/**
 * 日志等级
 */
export
enum log_level
{
		KDEBUG = 0, ///< DEBUG 等级，供调试使用
		KVERBOSE = 1, ///< VERBOSE 等级，用于输出一些繁杂信息
		KINFO = 2, ///< INFO 等级，用于输出一些通知性信息
		KWARNING = 3, ///< WARNING 等级，警告
		KERROR = 4, ///< ERROR 等级，表示遇到了一些错误，但是程序依旧可以带病运行
		KFATAL = 5, ///< FATAL 等级，表示极为严重的错误，一般遇到 FATAL 时程序已无法再继续运行
};


namespace kerbal
{

	namespace log
	{

		namespace detail
		{

			inline
			log_level & get_log_level_obj()
			{
				static log_level k_log_level = KINFO;
				return k_log_level;
			}
		}

		export
		inline
		void set_log_level(log_level level)
		{
			kerbal::log::detail::get_log_level_obj() = level;
		}

		export
		inline
		const char * log_level_description(log_level level)
		{
			switch (level) {
				case log_level::KDEBUG: {
					return "[DEBUG]";
				}
				case log_level::KVERBOSE: {
					return "[VERBO]";
				}
				case log_level::KINFO: {
					return "[INFO]";
				}
				case log_level::KWARNING: {
					return "[WARN]";
				}
				case log_level::KERROR: {
					return "[ERROR]";
				}
				case log_level::KFATAL: {
					return "[FATAL]";
				}
				default: {
					return "[OTHER]";
				}
			}
		}

		export
		inline
		kerbal::utility::costream::Color_t log_level_color(log_level level)
		{
			switch (level) {
				case log_level::KDEBUG: {
					return kerbal::utility::costream::LIGHT_PURPLE;
				}
				case log_level::KVERBOSE: {
					return kerbal::utility::costream::LIGHT_BLUE;
				}
				case log_level::KINFO: {
					return kerbal::utility::costream::GREEN;
				}
				case log_level::KWARNING: {
					return kerbal::utility::costream::YELLOW;
				}
				case log_level::KERROR: {
					return kerbal::utility::costream::LIGHT_RED;
				}
				case log_level::KFATAL: {
					return kerbal::utility::costream::RED;
				}
				default: {
					return kerbal::utility::costream::WHITE;
				}
			}
		}

		export
		inline constexpr
		char const * strip_prefix(char const * s, char const * prefix)
		{
			char const * its = s;
			while (*its != '\0' && *prefix != '\0') {
				if (*its != *prefix) {
					return s;
				}
				++its;
				++prefix;
			}
			if (*prefix == '\0') {
				return its;
			} else {
				return s;
			}
		}

		export
		template <typename Out>
		void log_write(Out & out, const char * src_file, int line, log_level level, std::string && info)
		{
			if (level < kerbal::log::detail::get_log_level_obj()) {
				return;
			}

			std::string s = fmt::format("{}  {:7}  {}:{}  {}",
										dksave::format_systime_to_datetime(std::chrono::system_clock::now()),
										kerbal::log::log_level_description(level),
										src_file, line, std::move(info));

			static std::mutex write_mtx;

			{
				std::lock_guard<std::mutex> guard(write_mtx);
				kerbal::utility::costream::costream<std::cout> ccout(log_level_color(level));
				ccout << s << std::endl;
			}
		}

	} // namespace log

} // namespace kerbal
