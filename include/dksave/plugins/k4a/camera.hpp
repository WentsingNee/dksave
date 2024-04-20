/**
 * @file       camera.hpp
 * @brief
 * @date       2022-10-07
 * @author     Wentsing Nee
 * @copyright
 *      Wentsing Nee of China Agricultural University
 *   all rights reserved
 */

#ifndef DKSAVE_PLUGINS_K4A_CAMERA_HPP
#define DKSAVE_PLUGINS_K4A_CAMERA_HPP

#include "context/k4a_img_color_to_cv_mat_context_t.hpp"
#include "context/k4a_img_depth_transform_to_color_mode_context_t.hpp"
#include "context/k4a_img_depth_to_cv_mat_context_t.hpp"

#include "dksave/logger.hpp"
#include "dksave/save_cv_mat.hpp"
#include "dksave/plugins/ucamera.hpp"

#include <string>
#include <filesystem>

// Kinect DK
#include <k4a/k4a.hpp>

#include <opencv2/core/mat.hpp>

// PCL
#if DKSAVE_ENABLE_PCL
#	include "context/k4a_img_depth_transform_to_point_cloud_mode_context_t.hpp"
#	include "context/k4a_img_point_cloud_to_pcl_point_cloud_context_t.hpp"

#	include <pcl/io/ply_io.h>
#endif


namespace dksave::plugins_k4a
{

	class capture_loop_context;

	class camera : public ucamera_base
	{
			using super = ucamera_base;

			k4a::device k_device;
			k4a_device_configuration_t k_config;

		public:
			using super::device_name;
			using super::enable;
			using super::previous_status;

			camera(
				k4a::device && device,
				std::string const & device_name,
				k4a_device_configuration_t config
			) :
				super(device_name),
				k_device(std::move(device)),
				k_config(std::move(config))
			{
			}

			k4a::device & device()
			{
				return k_device;
			}

			k4a_device_configuration_t const & config() const
			{
				return k_config;
			}

			void print_intrinsic() const
			{
				k4a_calibration_camera_t calib = this->k_device.get_calibration(
					this->k_config.depth_mode,
					this->k_config.color_resolution
				).depth_camera_calibration;

				auto s =
					fmt::format("device: {}\n", this->k_device.get_serialnum()) +
					fmt::format("resolution:\n"
								"    width: {}, height: {}\n",
								calib.resolution_width,
								calib.resolution_height
					) +
					fmt::format("principal point:\n"
								"    x: {}\n"
								"    y: {}\n",
								calib.intrinsics.parameters.param.cx,
								calib.intrinsics.parameters.param.cy
					) +
					fmt::format("focal length:\n"
								"    x: {}\n"
								"    y: {}\n",
								calib.intrinsics.parameters.param.fx,
								calib.intrinsics.parameters.param.fy
					) +
					fmt::format("radial distortion coefficients:\n") +
					fmt::format("    k1: {}\n", calib.intrinsics.parameters.param.k1) +
					fmt::format("    k2: {}\n", calib.intrinsics.parameters.param.k2) +
					fmt::format("    k3: {}\n", calib.intrinsics.parameters.param.k3) +
					fmt::format("    k4: {}\n", calib.intrinsics.parameters.param.k4) +
					fmt::format("    k5: {}\n", calib.intrinsics.parameters.param.k5) +
					fmt::format("    k6: {}\n", calib.intrinsics.parameters.param.k6) +
					fmt::format("center of distortion in Z=1 plane\n"
								"    x: {}\n"
								"    y: {}\n",
								calib.intrinsics.parameters.param.codx,
								calib.intrinsics.parameters.param.cody
					) +
					fmt::format("tangential distortion coefficient\n"
								"    x: {}\n"
								"    y: {}\n",
								calib.intrinsics.parameters.param.p1,
								calib.intrinsics.parameters.param.p2
					) +
					fmt::format("metric radius: {}\n", calib.intrinsics.parameters.param.metric_radius);
				KERBAL_LOG_WRITE(KINFO, "intrinsic:\n{}", s);
			}

			void start() try
			{
				k_device.start_cameras(&k_config);
				k_enable = true;
				KERBAL_LOG_WRITE(KINFO, "Start camera {}.", k_device_name);
				this->print_intrinsic();
			} catch (...) {
				k_enable = false;
				throw;
			}

			// 稳定化
			void stabilize() try
			{
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
							KERBAL_LOG_WRITE(KERROR, "Failed to give auto-exposure.", failed);
							this->stop();
							return;
						}
					}
				}
			} catch (...) {
				k_enable = false;
				throw;
			}

			void stop() noexcept
			{
				k_device.stop_cameras();
				k_enable = false;
			}

		public:

			using capture_loop_context = dksave::plugins_k4a::capture_loop_context;
	};


	class capture_loop_context
	{
			dksave::plugins_k4a::camera * camera;
			std::filesystem::path camera_working_dir;

			std::filesystem::path path_base_color = camera_working_dir / "rgb";
			std::filesystem::path path_base_depth = camera_working_dir / "depth";
			std::filesystem::path path_base_depth_clouds = camera_working_dir / "clouds";

			k4a_img_color_to_cv_mat_context_t k4a_img_color_to_cv_mat_context;

			k4a_img_depth_transform_to_color_mode_context_t k4a_img_depth_transform_to_color_mode_context;
			k4a_img_depth_to_cv_mat_context_t k4a_img_to_cv_mat_depth_context;

#if DKSAVE_ENABLE_PCL
			k4a_img_depth_transform_to_point_cloud_mode_context_t k4a_img_depth_transform_to_point_cloud_mode_context;
			k4a_img_point_cloud_to_pcl_point_cloud_context_t k4a_img_point_cloud_to_pcl_point_cloud_context;
#endif

			k4a::capture capture;
			k4a::transformation transformation;

		public:
			int frame_count = 0;

		public:
			capture_loop_context(class camera * camera, std::filesystem::path const & working_dir) :
				camera(camera),
				camera_working_dir(working_dir / this->camera->device_name()),
				transformation(
					camera->device().get_calibration(
						camera->config().depth_mode,
						camera->config().color_resolution
					)
				)
			{
			}


		private:
			static
			void describe_img(const k4a::image & img, const char type[])
			{
				KERBAL_LOG_WRITE(KDEBUG, "[{}]", type);
//				KERBAL_LOG_WRITE(KDEBUG, "format: {}", img.get_format());
				KERBAL_LOG_WRITE(KDEBUG, "device_timestamp: {}", img.get_device_timestamp().count());
				KERBAL_LOG_WRITE(KDEBUG, "system_timestamp: {}", img.get_system_timestamp().count());
				KERBAL_LOG_WRITE(KDEBUG, "height: {}, width: {}", img.get_height_pixels(), img.get_width_pixels());
			}

		public:
			void do_capture()
			{
				try {
					camera->device().get_capture(&capture);
				} catch (...) {
					KERBAL_LOG_WRITE(KERROR, "Camera {}: Get capture failed!", camera->device_name());
					throw;
				}
			}

			void handle_color(std::string const & date, std::string const & timestamp)
			{
				if (!capture) {
					return;
				}

				k4a::image k4a_img_color = capture.get_color_image();

				const cv::Mat & cv_color_img_without_alpha = k4a_img_color_to_cv_mat_context.convert(k4a_img_color);
				std::filesystem::path filename_color = path_base_color / date / (timestamp + ".png");
				try {
					save_cv_mat(cv_color_img_without_alpha, filename_color);
				} catch (...) {
					KERBAL_LOG_WRITE(
						KERROR, "Camera {}: color image saved failed: {}",
						camera->device_name(),
						filename_color.string()
					);
				}
			}

			void handle_depth(std::string const & date, std::string const & timestamp)
			{
				if (!capture) {
					return;
				}

				k4a::image k4a_img_depth = capture.get_depth_image();
				k4a::image * k4a_img_depth_transformed_to_color = nullptr;
				try {
					k4a_img_depth_transformed_to_color = &k4a_img_depth_transform_to_color_mode_context.transform(
						transformation, k4a_img_depth
					);
				} catch (...) {
					KERBAL_LOG_WRITE(
						KERROR, "Camera {}: depth image transformation to color failed",
						camera->device_name()
					);
					return;
				}

				const cv::Mat & cv_depth_img = k4a_img_to_cv_mat_depth_context.convert(
					*k4a_img_depth_transformed_to_color
				);
				std::filesystem::path filename_depth = path_base_depth / date / (timestamp + ".png");
				try {
					save_cv_mat(cv_depth_img, filename_depth);
				} catch (...) {
					KERBAL_LOG_WRITE(
						KERROR, "Camera {}: depth image saved failed: {}",
						camera->device_name(),
						filename_depth.string()
					);
				}

#if DKSAVE_ENABLE_PCL
				const k4a::image & k4a_img_point_cloud = k4a_img_depth_transform_to_point_cloud_mode_context.transform(transformation, *k4a_img_depth_transformed_to_color);
				const pcl::PointCloud<pcl::PointXYZ> & pcl_point_cloud = k4a_img_point_cloud_to_pcl_point_cloud_context.convert(k4a_img_point_cloud);
				std::filesystem::path filename_clouds = path_base_depth_clouds / date / (timestamp + ".ply");
				try {
					std::filesystem::create_directories(filename_clouds.parent_path());
					pcl::io::savePLYFile(filename_clouds.string(), pcl_point_cloud);
					KERBAL_LOG_WRITE(KINFO, "Saved {}", filename_clouds.string());
				} catch (...) {
					KERBAL_LOG_WRITE(
						KERROR, "Camera {}: depth clouds saved failed: {}",
						camera->device_name(),
						filename_depth.string()
					);
				}
#endif
			}

	};


	static_assert(dksave::ucamera<camera>, "k4a_camera doesn't meet the requirement of ucamera");


} // namespace dksave::plugins_k4a

#endif // DKSAVE_PLUGINS_K4A_CAMERA_HPP
