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
#include "context/k4a_img_color_transform_to_depth_mode_context_t.hpp"
#include "context/k4a_img_depth_transform_to_color_mode_context_t.hpp"
#include "context/k4a_img_depth_to_cv_mat_context_t.hpp"

#include "dksave/context/image_rotate_context_t.hpp"
#include "dksave/logger.hpp"
#include "dksave/save_cv_mat.hpp"
#include "dksave/plugins/ucamera.hpp"
#include "dksave/registration_mode_t.hpp"
#include "dksave/rotate_flag_t.hpp"

#include <chrono>
#include <string>
#include <filesystem>
#include <fmt/format.h>

// Kinect DK
#include <k4a/k4a.hpp>

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

			camera(
				k4a::device && device,
				std::string const & device_name,
				rotate_flag_t rotate_flag,
				registration_mode_t registration_mode,
				k4a_device_configuration_t config
			) :
				super(device_name, rotate_flag, registration_mode),
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
			void stabilize(int failed_total = 10, int success_total = 30) try
			{
				k4a::capture capture;
				int success = 0;
				int failed = 0;

				while (true) {
					bool capture_success = true;
					try {
						KERBAL_LOG_WRITE(KINFO, "Capture in stabilization start. camera: {}", k_device_name);
						capture_success = k_device.get_capture(&capture, 1s); // return false when timeout
					} catch (...) {
						capture_success = false;
					}

					if (capture_success) {
						success++;
						KERBAL_LOG_WRITE(KINFO, "Capture in stabilization success ({}/{}). camera: {}", success, success_total, k_device_name);

						if (success >= success_total) {
							KERBAL_LOG_WRITE(KINFO, "Stabilization success. camera: {}", k_device_name);
							return;
						}
					} else {
						failed++;
						KERBAL_LOG_WRITE(KWARNING, "Capture in stabilization failed ({}/{}). camera: {}", failed, failed_total, k_device_name);

						if (failed >= failed_total) {
							throw std::runtime_error("k4a stable failed");
						}
					}
				}
			} catch (...) {
				KERBAL_LOG_WRITE(KERROR, "Stabilization failed, stop the camera. camera: {}", k_device_name);
				this->stop();
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
		public:
			dksave::plugins_k4a::camera * camera;

		private:
			k4a_img_color_transform_to_depth_mode_context_t k4a_img_color_transform_to_depth_mode_context;
			k4a_img_color_to_cv_mat_context_t k4a_img_color_to_cv_mat_context;
			image_rotate_context_t cv_mat_color_rotate_context;

			k4a_img_depth_transform_to_color_mode_context_t k4a_img_depth_transform_to_color_mode_context;
			k4a_img_depth_to_cv_mat_context_t k4a_img_to_cv_mat_depth_context;
			image_rotate_context_t cv_mat_depth_rotate_context;

#if DKSAVE_ENABLE_PCL
			k4a_img_depth_transform_to_point_cloud_mode_context_t k4a_img_depth_transform_to_point_cloud_mode_context;
			k4a_img_point_cloud_to_pcl_point_cloud_context_t k4a_img_point_cloud_to_pcl_point_cloud_context;
#endif

			k4a::capture capture;
			k4a::transformation transformation;

		public:
			int frame_count = 0;

		public:
			capture_loop_context(class camera * camera) :
				camera(camera),
				transformation(
					camera->device().get_calibration(
						camera->config().depth_mode,
						camera->config().color_resolution
					)
				)
			{
			}


		private:
			static void describe_img(const k4a::image & img, const char type[])
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
				using namespace std::chrono_literals;
				try {
					bool ret = camera->device().get_capture(&capture, 1s);
					if (false == ret) {
						KERBAL_LOG_WRITE(KERROR, "Get capture timeout. camera: {}", camera->device_name());
						throw std::runtime_error("Get capture timeout");
					}
				} catch (...) {
					KERBAL_LOG_WRITE(KERROR, "Get capture failed. camera: {}", camera->device_name());
					throw;
				}
			}

			void handle_color(std::filesystem::path const & filename_color)
			{
				if (!capture) {
					return;
				}

				if (camera->config().color_resolution == K4A_COLOR_RESOLUTION_OFF) {
					return;
				}

				k4a::image k4a_img_color = capture.get_color_image();
				k4a::image const * k4a_img_color_transformed = nullptr;
				if (camera->registration_mode() == registration_mode_t::COLOR_TO_DEPTH) {
					k4a::image k4a_img_depth = capture.get_depth_image();
					try {
						k4a_img_color_transformed = &k4a_img_color_transform_to_depth_mode_context.transform(
							transformation,
							k4a_img_depth,
							k4a_img_color
						);
					} catch (std::exception const & e) {
						KERBAL_LOG_WRITE(KERROR, "Transforming color image to depth failed. camera: {}, exception_type: {}, what: {}",
										 camera->device_name(),
										 typeid(e).name(),
										 e.what()
						);
						return;
					} catch (...) {
						KERBAL_LOG_WRITE(KERROR, "Transforming color image to depth failed. camera: {}, exception_type: unknown",
										 camera->device_name()
						);
						return;
					}
				} else {
					k4a_img_color_transformed = &k4a_img_color;
				}

				const cv::Mat & cv_color_img_without_alpha = k4a_img_color_to_cv_mat_context.convert(*k4a_img_color_transformed);
				const cv::Mat & cv_color_img_rotated = cv_mat_color_rotate_context.rotate(
					cv_color_img_without_alpha, camera->rotate_flag()
				);

				try {
					save_cv_mat(cv_color_img_rotated, filename_color);
				} catch (std::exception const & e) {
					KERBAL_LOG_WRITE(KERROR, "RGB image saved failed. camera: {}, filename: {}, exception_type: {}, what: {}",
									 camera->device_name(),
									 filename_color.string(),
									 typeid(e).name(),
									 e.what()
					);
				} catch (...) {
					KERBAL_LOG_WRITE(KERROR, "RGB image saved failed. camera: {}, filename: {}, exception_type: unknown",
									 camera->device_name(),
									 filename_color.string()
					);
				}
			}

			void handle_depth(
				std::filesystem::path const & filename_depth,
				std::filesystem::path const & filename_pcloud
			)
			{
				if (!capture) {
					return;
				}

				if (camera->config().depth_mode == K4A_DEPTH_MODE_OFF) {
					return;
				}

				k4a::image k4a_img_depth = capture.get_depth_image();
				k4a::image const * k4a_img_depth_transformed = nullptr;

				bool enable_registration = camera->registration_mode() == registration_mode_t::DEPTH_TO_COLOR || DKSAVE_ENABLE_PCL;
				if (camera->config().color_resolution == K4A_COLOR_RESOLUTION_OFF) {
					enable_registration = false;
				}
				if (enable_registration) {
					// 如需转化为点云，则必须先向可见光配准
					try {
						k4a_img_depth_transformed = &k4a_img_depth_transform_to_color_mode_context.transform(
							transformation,
							k4a_img_depth
						);
					} catch (std::exception const & e) {
						KERBAL_LOG_WRITE(KERROR, "Transforming depth image to color failed. camera: {}, exception_type: {}, what: {}",
										 camera->device_name(),
										 typeid(e).name(),
										 e.what()
						);
						return;
					} catch (...) {
						KERBAL_LOG_WRITE(KERROR, "Transforming depth image to color failed. camera: {}, exception_type: unknown",
										 camera->device_name()
						);
						return;
					}

					KERBAL_LOG_WRITE(KVERBOSE, "Transforming depth image to color type success. camera: {}",
									 camera->device_name()
					);
				}

				k4a::image const * k4a_img_depth_outputted = nullptr;
				if (
					camera->registration_mode() == registration_mode_t::DEPTH_TO_COLOR &&
					nullptr != k4a_img_depth_transformed
				) {
					k4a_img_depth_outputted = k4a_img_depth_transformed;
				} else {
					k4a_img_depth_outputted = &k4a_img_depth;
				}

				const cv::Mat & cv_depth_img = k4a_img_to_cv_mat_depth_context.convert(*k4a_img_depth_outputted);
				const cv::Mat & cv_depth_img_rotated = cv_mat_depth_rotate_context.rotate(
					cv_depth_img, camera->rotate_flag()
				);

				try {
					save_cv_mat(cv_depth_img_rotated, filename_depth);
				} catch (std::exception const & e) {
					KERBAL_LOG_WRITE(KERROR, "Depth image saved failed. camera: {}, filename: {}, exception_type: {}, what: {}",
									 camera->device_name(),
									 filename_depth.string(),
									 typeid(e).name(),
									 e.what()
					);
				} catch (...) {
					KERBAL_LOG_WRITE(KERROR, "Depth image saved failed. camera: {}, filename: {}, exception_type: unknown",
									 camera->device_name(),
									 filename_depth.string()
					);
				}

#if DKSAVE_ENABLE_PCL
				if (nullptr != k4a_img_depth_transformed) {
					const k4a::image & k4a_img_point_cloud = k4a_img_depth_transform_to_point_cloud_mode_context.transform(transformation, *k4a_img_depth_transformed);
					const pcl::PointCloud<pcl::PointXYZ> & pcl_point_cloud = k4a_img_point_cloud_to_pcl_point_cloud_context.convert(k4a_img_point_cloud);
					try {
						std::filesystem::create_directories(filename_pcloud.parent_path());
						pcl::io::savePLYFile(filename_pcloud.string(), pcl_point_cloud);
						KERBAL_LOG_WRITE(KINFO, "Saved {}", filename_pcloud.string());
					} catch (...) {
						KERBAL_LOG_WRITE(KERROR, "Camera {}: depth clouds saved failed: {}", camera->device_name(), filename_depth.string());
					}
				}
#endif
			}

	};


	static_assert(dksave::ucamera<camera>, "k4a_camera doesn't meet the requirement of ucamera");


} // namespace dksave::plugins_k4a

#endif // DKSAVE_PLUGINS_K4A_CAMERA_HPP
