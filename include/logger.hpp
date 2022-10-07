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
#include <chrono>
#include <ctime>

#include <fmt/format.h>

#include "current_time.hpp"


/**
 * 日志等级
 */
enum log_level
{
		DEBUG = 0, ///< DEBUG 等级供调试使用，只在 DEBUG 模式下有效
		INFO, ///< INFO 等级，用于输出一些必要信息
		WARNING, ///< WARNING 等级，警告
		EROR, ///< ERROR 等级，表示遇到了一些错误，但是程序依旧可以带病运行
		FATAL, ///< FATAL 等级，表示极为严重的错误，程序发生了崩溃退出
};

namespace kerbal
{

	inline
	const char * log_level_description(log_level level)
	{
		switch (level) {
			case log_level::DEBUG: {
				return "[DEBUG]";
			}
			case log_level::INFO: {
				return "[INFO]";
			}
			case log_level::WARNING: {
				return "[WARNING]";
			}
			case log_level::EROR: {
				return "[ERROR]";
			}
			case log_level::FATAL: {
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
		if (level == DEBUG) {
			return;
		}
#   endif

		out
				<< fmt::format("{}  {:9}  {}:{}  {}",
							   current_time(),
							   log_level_description(level),
							   src_file, line, std::move(info))
				<< std::endl;
	}


} // namespace kerbal


#define KERBAL_LOG_WRITE(level, ...) do { \
    kerbal::log_write(std::cout, __FILE__, __LINE__, level, fmt::format(__VA_ARGS__)); \
} while(0)

#endif // KERBAL_LOGGER_HPP
