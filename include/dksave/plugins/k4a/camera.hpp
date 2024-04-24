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
#include "context/k4a_img_depth_transform_to_point_cloud_mode_context_t.hpp"
#include "context/k4a_img_point_cloud_to_pcl_point_cloud_context_t.hpp"

#include "dksave/logger.hpp"
#include "dksave/save_cv_mat.hpp"
#include "dksave/plugins/ucamera.hpp"

#include <chrono>
#include <string>
#include <filesystem>

// Kinect DK
#include <k4a/k4a.hpp>

#include <opencv2/core/mat.hpp>

// PCL
#if DKSAVE_ENABLE_PCL
#	include <pcl/io/ply_io.h>
#endif


template<>
struct fmt::formatter<k4a_calibration_type_t> :
	public fmt::formatter<std::string>
{
		static
		char const *
		k4a_calibration_type_t2_str(k4a_calibration_type_t c)
		{
			switch (c) {
				case K4A_CALIBRATION_TYPE_UNKNOWN: return "K4A_CALIBRATION_TYPE_UNKNOWN";
				case K4A_CALIBRATION_TYPE_DEPTH: return "K4A_CALIBRATION_TYPE_DEPTH";
				case K4A_CALIBRATION_TYPE_COLOR: return "K4A_CALIBRATION_TYPE_COLOR";
				case K4A_CALIBRATION_TYPE_GYRO: return "K4A_CALIBRATION_TYPE_GYRO";
				case K4A_CALIBRATION_TYPE_ACCEL: return "K4A_CALIBRATION_TYPE_ACCEL";
				case K4A_CALIBRATION_TYPE_NUM: return "K4A_CALIBRATION_TYPE_NUM";
			}
			return "ERROR";
		}

	public:
		auto format(k4a_calibration_type_t c, format_context & ctx) const
		{
			return fmt::formatter<std::string>::format(k4a_calibration_type_t2_str(c), ctx);
		}
};


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

		private:
			std::string k_format_intrinsic(k4a_calibration_camera_t const & calib) const
			{
				std::string s;

				fmt::format_to(
					std::back_inserter(s),
					"        resolution: {{\n"
					"            width: {}\n"
					"            height: {}\n"
					"        }}\n",
					calib.resolution_width,
					calib.resolution_height
				);
				fmt::format_to(
					std::back_inserter(s),
					"        principal point: {{\n"
					"            x: {}\n"
					"            y: {}\n"
					"        }}\n",
					calib.intrinsics.parameters.param.cx,
					calib.intrinsics.parameters.param.cy
				);
				fmt::format_to(
					std::back_inserter(s),
					"        focal length: {{\n"
					"            x: {}\n"
					"            y: {}\n"
					"        }}\n",
					calib.intrinsics.parameters.param.fx,
					calib.intrinsics.parameters.param.fy
				);
				fmt::format_to(
					std::back_inserter(s),
					"        radial distortion coefficients: {{\n"
					"            k1: {}\n"
					"            k2: {}\n"
					"            k3: {}\n"
					"            k4: {}\n"
					"            k5: {}\n"
					"            k6: {}\n"
					"        }}\n",
					calib.intrinsics.parameters.param.k1,
					calib.intrinsics.parameters.param.k2,
					calib.intrinsics.parameters.param.k3,
					calib.intrinsics.parameters.param.k4,
					calib.intrinsics.parameters.param.k5,
					calib.intrinsics.parameters.param.k6
				);
				fmt::format_to(
					std::back_inserter(s),
					"        center of distortion in Z=1 plane: {{\n"
					"            x: {}\n"
					"            y: {}\n"
					"        }}\n",
					calib.intrinsics.parameters.param.codx,
					calib.intrinsics.parameters.param.cody
				);
				fmt::format_to(
					std::back_inserter(s),
					"        tangential distortion coefficient: {{\n"
					"            x: {}\n"
					"            y: {}\n"
					"        }}\n",
					calib.intrinsics.parameters.param.p1,
					calib.intrinsics.parameters.param.p2
				);
				fmt::format_to(
					std::back_inserter(s),
					"        metric radius: {}\n",
					calib.intrinsics.parameters.param.metric_radius
				);
				return s;
			}

			std::string k_format_extrinsics(
				k4a_calibration_extrinsics_t const (&matrix)[K4A_CALIBRATION_TYPE_NUM][K4A_CALIBRATION_TYPE_NUM]
			) const
			{
				std::string s = "        extrinsics_index:\n";
				for (int i = 0; i < K4A_CALIBRATION_TYPE_NUM; ++i) {
					fmt::format_to(
						std::back_inserter(s),
						"            {}: {}\n", i, k4a_calibration_type_t(i)
					);
				}
				for (int i = 0; i < K4A_CALIBRATION_TYPE_NUM; ++i) {
					for (int j = 0; j < K4A_CALIBRATION_TYPE_NUM; ++j) {
						auto const & e = matrix[i][j];
						auto format_rotation = [&]() {
							auto r = e.rotation;
							fmt::format_to(
								std::back_inserter(s),
								"            rotation: {{\n"
								"                {{{}, {}, {}}},\n"
								"                {{{}, {}, {}}},\n"
								"                {{{}, {}, {}}}\n"
								"            }}\n",
								r[0], r[1], r[2],
								r[3], r[4], r[5],
								r[6], r[7], r[8]
							);
						};
						fmt::format_to(
							std::back_inserter(s),
							"        ({}, {})\n",
							i, j
						);
						format_rotation();
						fmt::format_to(
							std::back_inserter(s),
							"            translation: {{{}, {}, {}}} T\n",
							e.translation[0],
							e.translation[1],
							e.translation[2]
						);
					}
				}
				return s;
			}

		public:

			void print_calibration_information() const
			{
				auto calib = this->k_device.get_calibration(
					this->k_config.depth_mode,
					this->k_config.color_resolution
				);
				k4a_calibration_camera_t color_calib = calib.color_camera_calibration;
				k4a_calibration_camera_t depth_calib = calib.depth_camera_calibration;
				auto const & extrinsics = calib.extrinsics;

				std::string s;
				fmt::format_to(
					std::back_inserter(s),
					"calibration information: {{\n"
					"    device: {}\n"
					"    color_camera_calibration intrinsic: {{\n"
					"{}"
					"    }}\n"
					"    depth_camera_calibration intrinsic: {{\n"
					"{}"
					"    }}\n"
					"    extrinsics: {{\n"
					"{}"
					"    }}\n"
					"}}",
					this->k_device.get_serialnum(),
					this->k_format_intrinsic(color_calib),
					this->k_format_intrinsic(depth_calib),
					this->k_format_extrinsics(extrinsics)
				);
				KERBAL_LOG_WRITE(KINFO, "\n{}", s);
			}

			void start() try
			{
				k_device.start_cameras(&k_config);
				k_enable = true;
				KERBAL_LOG_WRITE(KINFO, "Start camera {}.", k_device_name);
				this->print_calibration_information();
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
			k4a_img_color_to_cv_mat_context_t k4a_img_color_to_cv_mat_context;

			k4a_img_depth_transform_to_color_mode_context_t k4a_img_depth_transform_to_color_mode_context;
			k4a_img_depth_to_cv_mat_context_t k4a_img_to_cv_mat_depth_context;
			k4a_img_depth_transform_to_point_cloud_mode_context_t k4a_img_depth_transform_to_point_cloud_mode_context;

#if DKSAVE_ENABLE_PCL
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

				k4a::image k4a_img_color = capture.get_color_image();

				const cv::Mat & cv_color_img_without_alpha = k4a_img_color_to_cv_mat_context.convert(k4a_img_color);
				try {
					save_cv_mat(cv_color_img_without_alpha, filename_color);
				} catch (std::exception const & e) {
					KERBAL_LOG_WRITE(
						KERROR, "Color image saved failed. camera: {}, filename: {}, exception_type: {}, what: {}",
						camera->device_name(),
						filename_color.string(),
						typeid(e).name(), e.what()
					);
				} catch (...) {
					KERBAL_LOG_WRITE(
						KERROR, "Color image saved failed. camera: {}, filename: {}, exception_type: unknown",
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

				k4a::image k4a_img_depth = capture.get_depth_image();
				k4a::image * k4a_img_depth_transformed_to_color = nullptr;
				try {
					k4a_img_depth_transformed_to_color = &k4a_img_depth_transform_to_color_mode_context.transform(
						transformation, k4a_img_depth);
				} catch (std::exception const & e) {
					KERBAL_LOG_WRITE(
						KERROR, "Depth image transformation to color failed. camera: {}, exception_type: {}, what: {}",
						camera->device_name(),
						typeid(e).name(), e.what()
					);
					return;
				} catch (...) {
					KERBAL_LOG_WRITE(
						KERROR, "Depth image transformation to color failed. camera: {}, exception_type: unknown",
						camera->device_name()
					);
					return;
				}

				const cv::Mat & cv_depth_img = k4a_img_to_cv_mat_depth_context.convert(
					*k4a_img_depth_transformed_to_color
				);
				try {
					save_cv_mat(cv_depth_img, filename_depth);
				} catch (std::exception const & e) {
					KERBAL_LOG_WRITE(
						KERROR, "Depth image saved failed. camera: {}, filename: {}, exception_type: {}, what: {}",
						camera->device_name(),
						filename_depth.string(),
						typeid(e).name(), e.what()
					);
				} catch (...) {
					KERBAL_LOG_WRITE(
						KERROR, "Depth image saved failed. camera: {}, filename: {}, exception_type: unknown",
						camera->device_name(),
						filename_depth.string()
					);
				}

#if DKSAVE_ENABLE_PCL
				const k4a::image & k4a_img_point_cloud = k4a_img_depth_transform_to_point_cloud_mode_context.transform(transformation, *k4a_img_depth_transformed_to_color);
				const pcl::PointCloud<pcl::PointXYZ> & pcl_point_cloud = k4a_img_point_cloud_to_pcl_point_cloud_context.convert(k4a_img_point_cloud);
				try {
					std::filesystem::create_directories(filename_pcloud.parent_path());
					pcl::io::savePLYFile(filename_pcloud.string(), pcl_point_cloud);
					KERBAL_LOG_WRITE(KINFO, "Saved {}", filename_pcloud.string());
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
