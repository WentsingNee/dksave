/**
 * @file       camera.hpp
 * @brief
 * @date       2023-09-24
 * @author     Wentsing Nee
 * @copyright
 *      Wentsing Nee of China Agricultural University
 *   all rights reserved
 */

#ifndef DKSAVE_PLUGINS_OB_CAMERA_HPP
#define DKSAVE_PLUGINS_OB_CAMERA_HPP

#include "config.hpp"
#include "context/H264_to_cv_mat_context.hpp"

#include "dksave/logger.hpp"
#include "dksave/save_cv_mat.hpp"
#include "dksave/plugins/ucamera.hpp"

#include <iostream>
#include <memory>
#include <string>
#include <filesystem>

#include <cstdlib>

#include <libobsensor/ObSensor.hpp>
#include <opencv2/core.hpp>
#include <fmt/format.h>
#include <fmt/ranges.h>


namespace dksave::plugins_ob
{

	class capture_loop_context;

	class camera : private ucamera_base
	{
			using super = ucamera_base;

			std::shared_ptr<ob::Device> k_device;
			std::shared_ptr<ob::Config> config;
			std::shared_ptr<ob::Pipeline> pipeline;

		public:
			using super::device_name;
			using super::enable;

		private:
			static
			void show_profiles_supported(std::shared_ptr<ob::StreamProfileList> profile_list)
			{
				for (uint32_t i = 0; i < profile_list->count(); ++i) {
					auto profile = std::const_pointer_cast<ob::StreamProfile>(
						profile_list->getProfile(i)
					)->as<ob::VideoStreamProfile>();
					if (profile) {
						std::cout << fmt::format("{}", *profile) << std::endl;
					} else {
						std::cout << fmt::format("nil") << std::endl;
					}
				}
			}

			void enable_color_stream(std::shared_ptr<ob::Config> config, ob_camera_configuration const & dksave_config)
			{
				KERBAL_LOG_WRITE(
					KINFO, "Enabling camera's color stream. camera: {}, config: {}",
					this->device_name(), dksave_config
				);
				auto profile_list = pipeline->getStreamProfileList(OB_SENSOR_COLOR);
				show_profiles_supported(profile_list);
				std::shared_ptr<ob::VideoStreamProfile> profile = nullptr;
				try {
					profile = profile_list->getVideoStreamProfile(
						dksave_config.width,
						dksave_config.height,
						dksave_config.format,
						dksave_config.fps
					);
				} catch (ob::Error const & e) {
					KERBAL_LOG_WRITE(
						KFATAL, "Profile not supported. camera: {}, config: {}, what: {}",
						this->device_name(), dksave_config, e.getMessage()
					);
					exit(EXIT_FAILURE);
				}
				if (profile) {
					config->enableStream(profile);
					KERBAL_LOG_WRITE(KINFO, "Enable color stream with profile: {}", *profile);
				}
			}

			void enable_depth_stream(std::shared_ptr<ob::Config> config, ob_camera_configuration const & dksave_config)
			{
				KERBAL_LOG_WRITE(
					KINFO, "Enabling camera's depth stream. camera: {}, config: {}",
					this->device_name(),
					dksave_config
				);
				auto profile_list = pipeline->getStreamProfileList(OB_SENSOR_DEPTH);
				show_profiles_supported(profile_list);
				std::shared_ptr<ob::VideoStreamProfile> profile = nullptr;
				try {
					profile = profile_list->getVideoStreamProfile(
						dksave_config.width,
						dksave_config.height,
						dksave_config.format,
						dksave_config.fps
					);
				} catch (ob::Error const & e) {
					KERBAL_LOG_WRITE(
						KERROR, "Profile not supported. camera: {}, config: {}, what: {}",
						this->device_name(), dksave_config, e.getMessage()
					);
				}
				if (profile) {
					config->enableStream(profile);
					KERBAL_LOG_WRITE(KINFO, "Enable depth stream with profile: {}", *profile);
				}
			}

		public:
			camera(
				std::shared_ptr<ob::Device> && device,
				std::string const & device_name,
				ob_camera_configuration const & config_rgb,
				ob_camera_configuration const & config_depth
			) :
				super(device_name),
				k_device(std::move(device)),
				config(std::make_shared<ob::Config>()),
				pipeline(std::make_shared<ob::Pipeline>(this->device()))
			{
				KERBAL_LOG_WRITE(KINFO, "Creating camera. camera: {}", this->device_name());
				this->enable_color_stream(config, config_rgb);
				this->enable_depth_stream(config, config_depth);
				config->setAlignMode(ALIGN_D2C_HW_MODE);
				config->setDepthScaleRequire(true);
			}

			std::shared_ptr<ob::Device> & device()
			{
				return k_device;
			}

		private:
			static
			void k_format_intrinsic(std::string & s, auto const & intrinsic, char const * header)
			{
				fmt::format_to(
					std::back_inserter(s),
					"	{}:\n"
					"		fx: {}\n"
					"		fy: {}\n"
					"		cx: {}\n"
					"		cy: {}\n"
					"		width: {}\n"
					"		height: {}\n",
					header,
					intrinsic.fx,
					intrinsic.fy,
					intrinsic.cx,
					intrinsic.cy,
					intrinsic.width,
					intrinsic.height
				);
			}

			static
			void k_format_distortion(std::string & s, auto const & distortion, char const * header)
			{
				fmt::format_to(
					std::back_inserter(s),
					"	{}:\n"
					"		k1: {}\n"
					"		k2: {}\n"
					"		k3: {}\n"
					"		k4: {}\n"
					"		k5: {}\n"
					"		k6: {}\n"
					"		p1: {}\n"
					"		p2: {}\n",
					header,
					distortion.k1,
					distortion.k2,
					distortion.k3,
					distortion.k4,
					distortion.k5,
					distortion.k6,
					distortion.p1,
					distortion.p2
				);
			}

		public:

			void print_calibration_information() const
			{
				auto calib = this->k_device->getCalibrationCameraParamList();
				auto paramGroupCount = calib->count();
				auto s =
					fmt::format("device: {}\n", this->k_device->getDeviceInfo()->serialNumber())
				;

				for (auto i = 0u; i < paramGroupCount; ++i) {
					auto cameraParam = calib->getCameraParam(i);
					auto transform = cameraParam.transform;

					fmt::format_to(std::back_inserter(s), "the {}-st param:\n", i);
					fmt::format_to(std::back_inserter(s), "	isMirrored: {}\n", cameraParam.isMirrored);

					k_format_intrinsic(s, cameraParam.rgbIntrinsic, "rgbIntrinsic");
					k_format_distortion(s, cameraParam.rgbDistortion, "rgbDistortion");
					k_format_intrinsic(s, cameraParam.depthIntrinsic, "depthIntrinsic");
					k_format_distortion(s, cameraParam.depthDistortion, "depthDistortion");

					fmt::format_to(
						std::back_inserter(s),
						"	transform:\n"
						"		rot: {}\n"
						"		trans: {}\n",
						transform.rot,
						transform.trans
					);
				}
				KERBAL_LOG_WRITE(KINFO, "intrinsic:\n{}", s);
			}

			void start()
			{
				pipeline->start(config);
				this->k_enable = true;
				this->print_calibration_information();
			}

			void stabilize()
			{
				for (int i = 0; i < 30; ++i) {
					auto frame_set = this->pipeline->waitForFrames(2000);
					if (frame_set == nullptr) {
						KERBAL_LOG_WRITE(KERROR, "Get capture failed. camera: {}", this->device_name());
						throw std::runtime_error("Get capture failed");
					}
				}
			}

			void stop() noexcept try
			{
				pipeline->stop();
				this->k_enable = false;
			} catch (...) {
				this->k_enable = false;
			}

			using capture_loop_context = dksave::plugins_ob::capture_loop_context;
			friend capture_loop_context;
	};


	class capture_loop_context
	{
		public:
			dksave::plugins_ob::camera * camera;

		private:
			std::shared_ptr<ob::FrameSet> frame_set;

			struct RGB_frame_to_cv_mat_context
			{
					cv::Mat cv_mat;

					cv::Mat const & cast(std::shared_ptr<ob::ColorFrame> color_frame)
					{
						this->cv_mat = cv::Mat(
							color_frame->height(),
							color_frame->width(),
							CV_8UC3,
							color_frame->data()
						);
						cv::cvtColor(this->cv_mat, this->cv_mat, cv::COLOR_RGB2BGR);
						return this->cv_mat;
					}
			} rgb_context;

			H264_to_cv_mat_context h264_context;

		public:
			int frame_count = 0;

		public:
			capture_loop_context(class camera * camera) :
				camera(camera)
			{
			}

			void do_capture()
			{
				frame_set = camera->pipeline->waitForFrames(2000);
				if (frame_set == nullptr) {
					KERBAL_LOG_WRITE(KERROR, "Get capture failed. camera: {}", camera->device_name());
					throw std::runtime_error("Get capture failed");
				}
			}

		public:
			void handle_color(std::filesystem::path const & filename_rgb)
			{
				if (!frame_set) {
					return;
				}
				auto color_frame = frame_set->colorFrame();
				if (color_frame == nullptr) {
					KERBAL_LOG_WRITE(KERROR, "Get color frame failed. camera: {}", camera->device_name());
					throw std::runtime_error("OB get color frame failed.");
				}
				KERBAL_LOG_WRITE(
					KVERBOSE, "Get color frame success. camera: {}",
					camera->device_name()
				);

				cv::Mat const * color_mat = nullptr;
				OBFormat color_format = color_frame->format();
				switch (color_format) {
					case OB_FORMAT_H264: {
						KERBAL_LOG_WRITE(
							KDEBUG, "Color frame dataSize: {}",
							color_frame->dataSize()
						);
						color_mat = &h264_context.decode(
							reinterpret_cast<std::uint8_t *>(color_frame->data()),
							color_frame->dataSize()
						);
						KERBAL_LOG_WRITE(
							KVERBOSE, "Decode color frame from H.264 success. camera: {}",
							camera->device_name()
						);
						break;
					}
					case OB_FORMAT_RGB: {
						color_mat = &rgb_context.cast(color_frame);
						KERBAL_LOG_WRITE(
							KVERBOSE, "Convert ob::Frame to cv::Mat success. camera: {}",
							camera->device_name()
						);
						break;
					}
					default: {
						KERBAL_LOG_WRITE(
							KERROR, "Unexpected OB_FORMAT. camera: {}, format: {}",
							camera->device_name(), color_format
						);
						throw std::runtime_error("Unexpected OB_FORMAT.");
					}
				}
				if (nullptr == color_mat) {
					KERBAL_LOG_WRITE(
						KERROR, "color_mat is null. camera: {}",
						camera->device_name()
					);
					throw std::runtime_error("color_mat is null.");
				}
				try {
					save_cv_mat(*color_mat, filename_rgb);
				} catch (std::exception const & e) {
					KERBAL_LOG_WRITE(
						KERROR, "Color image saved failed. camera: {}, filename_rgb: {}, exception type: {}, what: {}",
						camera->device_name(),
						filename_rgb.string(),
						typeid(e).name(), e.what()
					);
					throw;
				} catch (...) {
					KERBAL_LOG_WRITE(
						KERROR, "Color image saved failed. camera: {}, filename_rgb: {}, exception type: unknown",
						camera->device_name(),
						filename_rgb.string()
					);
					throw;
				}
			}

			void handle_depth(
				std::filesystem::path const & filename_depth,
				std::filesystem::path const & /*filename_pcloud*/
			)
			{
				if (!frame_set) {
					return;
				}
				auto depth_frame = frame_set->depthFrame();
				if (depth_frame == nullptr) {
					KERBAL_LOG_WRITE(KERROR, "Get depth frame failed. camera: {}", camera->device_name());
					return;
				}
				KERBAL_LOG_WRITE(
					KVERBOSE, "Get depth frame success. camera: {}, depth_frame.format: {}",
					camera->device_name(),
					depth_frame->format()
				);

				cv::Mat depth_mat(
					depth_frame->height(),
					depth_frame->width(),
					CV_16UC1,
					depth_frame->data()
				);
				try {
					save_cv_mat(depth_mat, filename_depth);
				} catch (std::exception const & e) {
					KERBAL_LOG_WRITE(
						KERROR, "Depth image saved failed. camera: {}, filename_rgb: {}, exception type: {}, what: {}",
						camera->device_name(),
						filename_depth.string(),
						typeid(e).name(), e.what()
					);
					throw;
				} catch (...) {
					KERBAL_LOG_WRITE(
						KERROR, "Depth image saved failed. camera: {}, filename_depth: {}, exception type: unknown",
						camera->device_name(),
						filename_depth.string()
					);
					throw;
				}
			}

	};


	static_assert(
		dksave::ucamera<camera>,
		"ob_camera doesn't meet the requirement of ucamera"
	);


} // namespace dksave::plugins_ob

#endif // DKSAVE_PLUGINS_OB_CAMERA_HPP
