/**
 * @file       k4a_camera.hpp
 * @brief
 * @date       2022-10-07
 * @author     Peter
 * @copyright
 *      Peter of [ThinkSpirit Laboratory](http://thinkspirit.org/)
 *   of [Nanjing University of Information Science & Technology](http://www.nuist.edu.cn/)
 *   all rights reserved
 */

#ifndef DKSAVE_K4A_CAMERA_HPP
#define DKSAVE_K4A_CAMERA_HPP

#include "context/k4a_context/k4a_img_color_to_cv_mat_context_t.hpp"
#include "context/k4a_context/k4a_img_depth_transform_to_color_mode_context_t.hpp"
#include "context/k4a_context/k4a_img_depth_to_cv_mat_context_t.hpp"
#include "context/k4a_context/k4a_img_depth_transform_to_point_cloud_mode_context_t.hpp"
#include "context/k4a_context/k4a_img_point_cloud_to_pcl_point_cloud_context_t.hpp"

#include "save_cv_mat.hpp"
#include "ucamera.hpp"
#include "logger.hpp"
#include "working_status.hpp"

#include <chrono>
#include <string>
#include <filesystem>
#include <thread>
#include <ctime>
#include <windows.h>

// Kinect DK
#include <k4a/k4a.hpp>

// PCL
#if DKSAVE_SUPPORT_PCL
#   include <pcl/io/ply_io.h>
#endif


class k4a_camera : public ucamera {
		using super = ucamera;

		k4a::device k_device;
		k4a_device_configuration_t k_config;

	public:
		k4a_camera(
				k4a::device &&device,
				std::string const &device_name,
				k4a_device_configuration_t &&config) :
				super(device_name),
				k_device(std::move(device)),
				k_config(std::move(config)) {
		}

		k4a::device &device() {
			return k_device;
		}

		k4a_device_configuration_t const &config() const {
			return k_config;
		}

		virtual void start() override try {
			k_device.start_cameras(&k_config);
			k_enable = true;
			KERBAL_LOG_WRITE(KINFO, "Start camera {}.", k_device_name);
		} catch (...) {
			k_enable = false;
			throw;
		}

		// 稳定化
		virtual void stabilize() override try {
			k4a::capture capture;
			int success = 0; //用来稳定，类似自动曝光
			int failed = 0; // 统计自动曝光的失败次数

			while (true) {
				bool capture_success = true;
				try {
					capture_success = k_device.get_capture(&capture);
				} catch (...) {
					capture_success = false;
				}

				if (capture_success) {
					// 跳过前 n 个（成功的数据采集）循环，用来稳定
					success++;
					KERBAL_LOG_WRITE(KINFO, "Capture several frames to give auto-exposure for {} times.", success);

					if (success >= 30) {
						KERBAL_LOG_WRITE(KINFO, "Done: auto-exposure.");
						return; // 完成相机的稳定过程
					}
				} else {
					failed++;
					KERBAL_LOG_WRITE(KWARNING, "K4A_WAIT_RESULT_TIMEOUT for {} times.", failed);

					if (failed >= 30) {
						KERBAL_LOG_WRITE(KFATAL, "Failed to give auto-exposure.", failed);
						this->stop();
						return;
					}
				}
			}
		} catch (...) {
			k_enable = false;
			throw;
		}

		virtual void stop() noexcept override {
			k_device.stop_cameras();
			k_enable = false;
		}

	private:
		static void describe_img(const k4a::image & img, const char type[])
		{
			KERBAL_LOG_WRITE(KDEBUG, "[{}]", type);
//			KERBAL_LOG_WRITE(KDEBUG, "format: {}", img.get_format());
			KERBAL_LOG_WRITE(KDEBUG, "device_timestamp: {}", img.get_device_timestamp().count());
			KERBAL_LOG_WRITE(KDEBUG, "system_timestamp: {}", img.get_system_timestamp().count());
			KERBAL_LOG_WRITE(KDEBUG, "height: {}, width: {}", img.get_height_pixels(), img.get_width_pixels());
		}

	public:
		virtual void working_loop(std::filesystem::path const & working_dir, std::chrono::milliseconds sleep_period) override {

			using namespace std::chrono_literals;

			std::filesystem::path camera_working_dir = working_dir / this->device_name();

			std::filesystem::path path_base_color = camera_working_dir / "rgb";
			std::filesystem::path path_base_depth = camera_working_dir / "depth";
			std::filesystem::path path_base_depth_clouds = camera_working_dir / "clouds";

			k4a_img_color_to_cv_mat_context_t k4a_img_color_to_cv_mat_context;
			k4a_img_depth_transform_to_color_mode_context_t k4a_img_depth_transform_to_color_mode_context;
			k4a_img_depth_to_cv_mat_context_t k4a_img_to_cv_mat_depth_context;
			k4a_img_depth_transform_to_point_cloud_mode_context_t k4a_img_depth_transform_to_point_cloud_mode_context;

#if DKSAVE_SUPPORT_PCL
			k4a_img_point_cloud_to_pcl_point_cloud_context_t k4a_img_point_cloud_to_pcl_point_cloud_context;
#endif

			k4a::capture capture;
			k4a::transformation transformation(
					this->device().get_calibration(
							this->config().depth_mode,
							this->config().color_resolution
					)
			);

			int count = 0;
			working_status previous_status = working_status::WORK;

			while (true) {

				auto start_time = std::chrono::system_clock::now();

				working_status status = get_working_status(start_time);

				if (status != previous_status) {
					KERBAL_LOG_WRITE(KINFO, "Switch status from {} to {}.", previous_status, status);
					previous_status = status;
				}

				if (status == working_status::SLEEP) {
					if (this->enable()) {
						KERBAL_LOG_WRITE(KINFO, "Camera {} fall in sleep.", this->device_name());
						this->stop();
					}
					std::this_thread::sleep_for(1min);
					continue;
				} else {
					if (!this->enable()) {
						// 启动设备
						KERBAL_LOG_WRITE(KINFO, "Camera {} is sleeping, try to wake up...", this->device_name());
						try {
							this->start();
							KERBAL_LOG_WRITE(KINFO, "Camera {} has been started.", this->device_name());
						} catch (...) {
							KERBAL_LOG_WRITE(KERROR, "Waking up camara {} failed.", this->device_name());
							std::this_thread::sleep_for(30s);
							continue;
						}
						try {
							this->stabilize();
							KERBAL_LOG_WRITE(KINFO, "Camera {} has been stable.", this->device_name());
						} catch (...) {
							KERBAL_LOG_WRITE(KERROR, "Stabilization process of camara {} failed.",
											 this->device_name());
							std::this_thread::sleep_for(30s);
							continue;
						}
					}
				}


				try {

					bool capture_success = false;
					try {
						capture_success = this->device().get_capture(&capture);
					} catch (...) {
						KERBAL_LOG_WRITE(KERROR, "Camera {}: Get capture failed!", this->device_name());
						std::this_thread::sleep_for(500ms);
						continue;
					}

					if (!capture_success) {
						KERBAL_LOG_WRITE(KERROR, "Camera {}: Get capture failed!", this->device_name());
						std::this_thread::sleep_for(500ms);
						continue;
					}

					SYSTEMTIME time;
					GetLocalTime(&time);

					std::string date = format_systime_to_date(time);
					std::string timestamp = format_systime_to_timestamp(time);

					k4a::image k4a_img_color = capture.get_color_image();

					const cv::Mat &cv_color_img_without_alpha = k4a_img_color_to_cv_mat_context.convert(k4a_img_color);
					std::filesystem::path filename_color = path_base_color / date / (timestamp + ".png");
					try {
						save_cv_mat(cv_color_img_without_alpha, filename_color);
					} catch (...) {
						KERBAL_LOG_WRITE(KERROR, "Camera {}: color image saved failed: {}", this->device_name(),
										 filename_color.string());
					}

					bool handle_depth = true;
					k4a::image k4a_img_depth = capture.get_depth_image();
					k4a::image *k4a_img_depth_transformed_to_color = nullptr;
					try {
						k4a_img_depth_transformed_to_color = &k4a_img_depth_transform_to_color_mode_context.transform(
								transformation, k4a_img_depth);
					} catch (...) {
						KERBAL_LOG_WRITE(KERROR, "Camera {}: depth image transformation to color failed",
										 this->device_name());
						handle_depth = false;
					}

					if (handle_depth) {
						const cv::Mat &cv_depth_img = k4a_img_to_cv_mat_depth_context.convert(
								*k4a_img_depth_transformed_to_color);
						std::filesystem::path filename_depth = path_base_depth / date / (timestamp + ".png");
						try {
							save_cv_mat(cv_depth_img, filename_depth);
						} catch (...) {
							KERBAL_LOG_WRITE(KERROR, "Camera {}: depth image saved failed: {}", this->device_name(),
											 filename_depth.string());
						}

#if DKSAVE_SUPPORT_PCL
						const k4a::image & k4a_img_point_cloud = k4a_img_depth_transform_to_point_cloud_mode_context.transform(transformation, *k4a_img_depth_transformed_to_color);
						const pcl::PointCloud<pcl::PointXYZ> & pcl_point_cloud = k4a_img_point_cloud_to_pcl_point_cloud_context.convert(k4a_img_point_cloud);
						std::filesystem::path filename_clouds = path_base_depth_clouds / date / (timestamp + ".ply");
						try {
							std::filesystem::create_directories(filename_clouds.parent_path());
							pcl::io::savePLYFile(filename_clouds.string(), pcl_point_cloud);
							KERBAL_LOG_WRITE(KINFO, "Saved {}", filename_clouds.string());
						} catch (...) {
							KERBAL_LOG_WRITE(KERROR, "Camera {}: depth clouds saved failed: {}", this->device_name(), filename_depth.string());
						}
#endif

					}

					count++;
					KERBAL_LOG_WRITE(KINFO, "Camera {}: Frame {} handled done.", this->device_name(), count);

				} catch (...) {
					KERBAL_LOG_WRITE(KERROR, "Camera {}: Unhandled exception.", this->device_name());
				}

				using namespace std::chrono_literals;
				std::this_thread::sleep_until(start_time + sleep_period);

			} // while

		}
};

#endif // DKSAVE_K4A_CAMERA_HPP
