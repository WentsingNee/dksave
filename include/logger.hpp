/**
 * @file       logger.hpp
 * @brief
 * @date       2022-10-06
 * @author     Peter
 * @copyright
 *      Peter of [ThinkSpirit Laboratory](http://thinkspirit.org/)
 *   of [Nanjing University of Information Science & Technology](http://www.nuist.edu.cn/)
 *   all rights reserved
 */

#ifndef KERBAL_LOGGER_HPP
#define KERBAL_LOGGER_HPP

#include <iostream>
#include <mutex>

#include <ctime>

#include <fmt/format.h>

#include "current_time.hpp"

/**
 * 日志等级
 */
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

		inline
		void set_log_level(log_level level)
		{
			kerbal::log::detail::get_log_level_obj() = level;
		}

		inline
		const char * log_level_description(log_level level)
		{
			switch (level) {
				case log_level::KDEBUG: {
					return "[DEBUG]";
				}
				case log_level::KVERBOSE: {
					return "[VERBOSE]";
				}
				case log_level::KINFO: {
					return "[INFO]";
				}
				case log_level::KWARNING: {
					return "[WARNING]";
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

		template <typename Out>
		void log_write(Out & out, const char * src_file, int line, log_level level, std::string && info)
		{
			if (level < kerbal::log::detail::get_log_level_obj()) {
				return;
			}

			SYSTEMTIME time;
			GetLocalTime(&time);
			std::string s = fmt::format("{}  {:9}  {}:{}  {}",
										format_systime_to_time(time),
										kerbal::log::log_level_description(level),
										src_file, line, std::move(info));

//			auto now = std::chrono::system_clock::now();
//			std::string s = fmt::format("{}  {:9}  {}:{}  {}",
//										format_time_point_to_time(now),
//										log_level_description(level),
//										src_file, line, std::move(info));

			static std::mutex write_mtx;

			{
				std::lock_guard<std::mutex> guard(write_mtx);
				out << s << std::endl;
			}
		}

	} // namespace log

} // namespace kerbal


#define KERBAL_LOG_WRITE(level, ...) do { \
	kerbal::log::log_write(std::cout, __FILE__, __LINE__, level, fmt::format(__VA_ARGS__)); \
} while(0)

#endif // KERBAL_LOGGER_HPP
