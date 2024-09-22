/**
 * @file       logger.hpp
 * @brief
 * @date       2024-09-22
 * @author     Peter
 * @copyright
 *      Wentsing Nee of China Agricultural University
 *   all rights reserved
 */

#ifndef DKSAVE_LOGGER_HPP
#define DKSAVE_LOGGER_HPP

#include <iostream>

#include <fmt/format.h>


#ifdef SOURCE_ROOT
#define KERBAL_LOG_WRITE(level, ...) do { \
	kerbal::log::log_write(std::cout, kerbal::log::strip_prefix(__FILE__, SOURCE_ROOT), __LINE__, level, fmt::format(__VA_ARGS__)); \
} while(0)
#else
#define KERBAL_LOG_WRITE(level, ...) do { \
	kerbal::log::log_write(std::cout, __FILE__, __LINE__, level, fmt::format(__VA_ARGS__)); \
} while(0)
#endif

#endif // DKSAVE_LOGGER_HPP
