/**
 * @file       global_settings.hpp
 * @brief
 * @date       2024-04-10
 * @author     Wentsing Nee
 * @copyright
 *      Wentsing Nee of China Agricultural University
 *   all rights reserved
 */

#ifndef DKSAVE_GLOBAL_SETTINGS_HPP
#define DKSAVE_GLOBAL_SETTINGS_HPP

#include "dksave/logger.hpp"
#include "dksave/working_status.hpp"

#include <kerbal/container/avl_set.hpp>

#include <filesystem>
#include <chrono>
#include <stdexcept>
#include <string>

#include <ctre.hpp>
#include <fmt/format.h>
#include <yaml-cpp/yaml.h>


namespace dksave
{

	class global_settings
	{

		private:
			static inline std::filesystem::path working_dir;
			static inline std::chrono::milliseconds sleep_period = 48ms;
			static inline std::chrono::milliseconds frame_handle_timeout_rgb = 150ms;
			static inline std::chrono::milliseconds frame_handle_timeout_depth = 150ms;
			static inline kerbal::container::avl_set<std::string> device_name_occupied;

		public:
			static
			auto const & get_working_dir()
			{
				return working_dir;
			}

			static
			auto const & get_sleep_period()
			{
				return sleep_period;
			}

			static
			auto const & get_frame_handle_timeout_rgb()
			{
				return frame_handle_timeout_rgb;
			}

			static
			auto const & get_frame_handle_timeout_depth()
			{
				return frame_handle_timeout_depth;
			}

			static
			auto add_device_name_occupied(std::string const & name)
			{
				return device_name_occupied.insert(name);
			}

		private:
			/**
			 * 将 hh:mm:ss 格式的字符串转换为 std::chrono::seconds
			 */
			static
			std::chrono::seconds parse_clock(std::string const & s)
			{
				constexpr const char CLOCK_PATTERN[] = R"(\d\d:\d\d:\d\d)";
				if (!ctre::match<CLOCK_PATTERN>(s)) {
					throw std::runtime_error(fmt::format("wrong clock format. expect format: <hh:mm:ss>, got: \"{}\"", s));
				}

				int hour = std::stoi(s.substr(0, 2));
				if (hour < 0 || hour >= 24) {
					throw std::runtime_error("unreasonable hour in clock");
				}
				int min = std::stoi(s.substr(3, 2));
				if (min < 0 || min >= 60) {
					throw std::runtime_error("unreasonable minute in clock");
				}
				int sec = std::stoi(s.substr(6, 2));
				if (sec < 0 || sec >= 60) {
					throw std::runtime_error("unreasonable second in clock");
				}
				return std::chrono::hours(hour) + std::chrono::minutes(min) + std::chrono::seconds(sec);
			}

		public:
			static
			void parse(YAML::Node const & config_node)
			{
				auto parse_field = [](char const * field, auto parse) {
					try {
						parse();
					} catch (YAML::InvalidNode const & e) {
						KERBAL_LOG_WRITE(KFATAL, "Parse {} failed. what: {}", field, e.what());
						throw;
					} catch (std::exception const & e) {
						KERBAL_LOG_WRITE(KFATAL, "Parse {} failed. what: {}", field, e.what());
						throw;
					}
				};

				parse_field("working_dir", [&config_node]() {
					std::string working_dir_str;
					working_dir_str = config_node["working_dir"].as<std::string>();
					working_dir = std::filesystem::path(working_dir_str);
					KERBAL_LOG_WRITE(KINFO, "Parse working_dir success. working_dir: {}", working_dir_str);
				});

				parse_field("sleep_period", [&config_node]() {
					int sleep_period_i = config_node["sleep_period"].as<int>();
					sleep_period = std::chrono::milliseconds(sleep_period_i);
					KERBAL_LOG_WRITE(KINFO, "Parse sleep_period success. sleep_period: {} ms", sleep_period.count());
				});

				parse_field("start_time", [&config_node]() {
					std::string s = config_node["start_time"].as<std::string>();
					start_time = parse_clock(s);
					KERBAL_LOG_WRITE(KINFO, "Parse start_time success. start_time: {}", s);
				});

				parse_field("end_time", [&config_node]() {
					std::string s = config_node["end_time"].as<std::string>();
					end_time = parse_clock(s);
					KERBAL_LOG_WRITE(KINFO, "Parse end_time success. end_time: {}", s);
				});


				YAML::Node frame_handle_timeout_node = config_node["frame_handle_timeout"];

				parse_field("frame_handle_timeout.rgb", [&frame_handle_timeout_node]() {
					int frame_handle_timeout_rgb_ = frame_handle_timeout_node["rgb"].as<int>();
					frame_handle_timeout_rgb = std::chrono::milliseconds(frame_handle_timeout_rgb_);
					KERBAL_LOG_WRITE(
						KINFO, "Parse frame_handle_timeout.rgb success. frame_handle_timeout.rgb: {}",
						frame_handle_timeout_rgb
					);
				});

				parse_field("frame_handle_timeout.depth", [&frame_handle_timeout_node]() {
					int frame_handle_timeout_depth_ = frame_handle_timeout_node["depth"].as<int>();
					frame_handle_timeout_depth = std::chrono::milliseconds(frame_handle_timeout_depth_);
					KERBAL_LOG_WRITE(
						KINFO, "Parse frame_handle_timeout.depth success. frame_handle_timeout.depth: {}",
						frame_handle_timeout_depth
					);
				});

				parse_field("log_level", [&config_node]() {
					std::string s = config_node["log_level"].as<std::string>();
					KERBAL_LOG_WRITE(KINFO, "Parse log_level success. log_level: {}", s);

					if (s == "KDEBUG") {
						kerbal::log::set_log_level(KDEBUG);
						return;
					}
					if (s == "KVERBOSE") {
						kerbal::log::set_log_level(KVERBOSE);
						return;
					}
					if (s == "KINFO") {
						kerbal::log::set_log_level(KINFO);
						return;
					}
					if (s == "KWARNING") {
						kerbal::log::set_log_level(KWARNING);
						return;
					}
					if (s == "KERROR") {
						kerbal::log::set_log_level(KERROR);
						return;
					}
					if (s == "KFATAL") {
						kerbal::log::set_log_level(KFATAL);
						return;
					}

					KERBAL_LOG_WRITE(KFATAL, "Unknown log level. log_level: {}", s);
					throw std::runtime_error("Unknown log level");
				});

			}

	};

} // namespace dksave

#endif // DKSAVE_GLOBAL_SETTINGS_HPP
