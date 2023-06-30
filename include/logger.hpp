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
		KDEBUG = 0, ///< DEBUG 等级供调试使用，只在 DEBUG 模式下有效
		KINFO = 1, ///< INFO 等级，用于输出一些必要信息
		KWARNING = 2, ///< WARNING 等级，警告
		KERROR = 3, ///< ERROR 等级，表示遇到了一些错误，但是程序依旧可以带病运行
		KFATAL = 4, ///< FATAL 等级，表示极为严重的错误，程序发生了崩溃退出
};

namespace kerbal
{

	inline
	const char * log_level_description(log_level level)
	{
		switch (level) {
			case log_level::KDEBUG: {
				return "[DEBUG]";
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

#   ifdef NDEBUG
		if (level == KDEBUG) {
			return;
		}
#   endif

		SYSTEMTIME time;
		GetLocalTime(&time);
		std::string s = fmt::format("{}  {:9}  {}:{}  {}",
									   format_systime_to_time(time),
									   log_level_description(level),
									   src_file, line, std::move(info));

		static std::mutex write_mtx;

		{
			std::lock_guard<std::mutex> guard(write_mtx);
			out << s << std::endl;
		}
	}


} // namespace kerbal


#define KERBAL_LOG_WRITE(level, ...) do { \
	kerbal::log_write(std::cout, __FILE__, __LINE__, level, fmt::format(__VA_ARGS__)); \
} while(0)

#endif // KERBAL_LOGGER_HPP
