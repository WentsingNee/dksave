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
#include "context/H264_to_cv_mat_context_t.hpp"
#include "context/ob_color_frame_to_cv_mat_context_t.hpp"
#include "context/ob_frame_depth_to_cv_mat_context_t.hpp"
#include "context/ob_img_depth_transform_to_color_mode_context_t.hpp"

#include "dksave/logger.hpp"
#include "dksave/save_cv_mat.hpp"
#include "dksave/plugins/ucamera.hpp"

#include <memory>
#include <string>
#include <filesystem>

#include <libobsensor/ObSensor.hpp>
#include <opencv2/core.hpp>


namespace dksave::plugins_ob
{

	class capture_loop_context;

	class camera : private ucamera_base
	{
			using super = ucamera_base;

			std::shared_ptr<ob::Device> k_device;
			std::shared_ptr<ob::Config> config;
			std::shared_ptr<ob::Pipeline> pipeline;
			ob_camera_configuration config_rgb;
			ob_camera_configuration config_depth;

		public:
			using super::device_name;
			using super::enable;

		private:
			static void show_profiles_supported(std::shared_ptr<ob::StreamProfileList> profile_list)
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
				KERBAL_LOG_WRITE(KINFO, "Enabling camera's color stream. camera: {}, config: {}", this->device_name(), dksave_config);
				auto profile_list = pipeline->getStreamProfileList(OB_SENSOR_COLOR);
				show_profiles_supported(profile_list);
				std::shared_ptr<ob::VideoStreamProfile> profile = nullptr;
				try {
					profile = profile_list->getVideoStreamProfile(dksave_config.width, dksave_config.height, dksave_config.format, dksave_config.fps);
				} catch (ob::Error const & e) {
					KERBAL_LOG_WRITE(KFATAL, "Profile not supported. camera: {}, config: {}, what: {}",
									 this->device_name(), dksave_config, e.getMessage());
					exit(EXIT_FAILURE);
				}
				if (profile) {
					config->enableStream(profile);
					KERBAL_LOG_WRITE(KINFO, "Enable color stream with profile: {}", *profile);
				}
			}

			void enable_depth_stream(std::shared_ptr<ob::Config> config, ob_camera_configuration const & dksave_config)
			{
				KERBAL_LOG_WRITE(KINFO, "Enabling camera's depth stream. camera: {}, config: {}", this->device_name(), dksave_config);
				auto profile_list = pipeline->getStreamProfileList(OB_SENSOR_DEPTH);
				show_profiles_supported(profile_list);
				std::shared_ptr<ob::VideoStreamProfile> profile = nullptr;
				try {
					profile = profile_list->getVideoStreamProfile(dksave_config.width, dksave_config.height, dksave_config.format, dksave_config.fps);
				} catch (ob::Error const & e) {
					KERBAL_LOG_WRITE(KERROR, "Profile not supported. camera: {}, config: {}, what: {}",
									 this->device_name(), dksave_config, e.getMessage());
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
				pipeline(std::make_shared<ob::Pipeline>(this->device())),
				config_rgb(config_rgb),
				config_depth(config_depth)
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

			void print_intrinsic() const
			{
				auto calib = this->k_device->getCalibrationCameraParamList();
				auto paramGroupCount = calib->count();
				auto s =
					fmt::format("device: {}\n", this->k_device->getDeviceInfo()->serialNumber())
				;

				for (auto i = 0u; i < paramGroupCount; ++i) {
					auto cameraParam = calib->getCameraParam(i);
					auto rgbIntrinsic = cameraParam.rgbIntrinsic;
					auto rgbDistortion = cameraParam.rgbDistortion;
					auto depthIntrinsic = cameraParam.depthIntrinsic;
					auto depthDistortion = cameraParam.depthDistortion;
					auto transform = cameraParam.transform;

					s +=
						fmt::format("the {}-st param:\n", i) +
						fmt::format("	isMirrored: {}\n", cameraParam.isMirrored) +
						fmt::format("	rgbIntrinsic:\n") +
						fmt::format("		fx: {}\n", rgbIntrinsic.fx) +
						fmt::format("		fy: {}\n", rgbIntrinsic.fy) +
						fmt::format("		cx: {}\n", rgbIntrinsic.cx) +
						fmt::format("		cy: {}\n", rgbIntrinsic.cy) +
						fmt::format("		width: {}\n", rgbIntrinsic.width) +
						fmt::format("		height: {}\n", rgbIntrinsic.height) +
						fmt::format("	rgbDistortion:\n") +
						fmt::format("		k1: {}\n", rgbDistortion.k1) +
						fmt::format("		k2: {}\n", rgbDistortion.k2) +
						fmt::format("		k3: {}\n", rgbDistortion.k3) +
						fmt::format("		k4: {}\n", rgbDistortion.k4) +
						fmt::format("		k5: {}\n", rgbDistortion.k5) +
						fmt::format("		k6: {}\n", rgbDistortion.k6) +
						fmt::format("		p1: {}\n", rgbDistortion.p1) +
						fmt::format("		p2: {}\n", rgbDistortion.p2) +
						fmt::format("	depthIntrinsic:\n") +
						fmt::format("		fx: {}\n", depthIntrinsic.fx) +
						fmt::format("		fy: {}\n", depthIntrinsic.fy) +
						fmt::format("		cx: {}\n", depthIntrinsic.cx) +
						fmt::format("		cy: {}\n", depthIntrinsic.cy) +
						fmt::format("		width: {}\n", depthIntrinsic.width) +
						fmt::format("		height: {}\n", depthIntrinsic.height) +
						fmt::format("	depthDistortion:\n") +
						fmt::format("		k1: {}\n", depthDistortion.k1) +
						fmt::format("		k2: {}\n", depthDistortion.k2) +
						fmt::format("		k3: {}\n", depthDistortion.k3) +
						fmt::format("		k4: {}\n", depthDistortion.k4) +
						fmt::format("		k5: {}\n", depthDistortion.k5) +
						fmt::format("		k6: {}\n", depthDistortion.k6) +
						fmt::format("		p1: {}\n", depthDistortion.p1) +
						fmt::format("		p2: {}\n", depthDistortion.p2) +
						fmt::format("	transform:\n") +
						fmt::format("		rot: {}\n", transform.rot) +
						fmt::format("		trans: {}\n", transform.trans)
					;
				}
				KERBAL_LOG_WRITE(KINFO, "intrinsic:\n{}", s);
			}

			void start()
			{
				pipeline->start(config);
				this->k_enable = true;
				this->print_intrinsic();
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

			ob_color_frame_to_cv_mat_context_t ob_color_frame_to_cv_mat_context;
			H264_to_cv_mat_context_t H264_to_cv_mat_context;
			ob_img_depth_transform_to_color_mode_context_t ob_img_depth_transform_to_color_mode_context;
			ob_frame_depth_to_cv_mat_context_t ob_frame_depth_to_cv_mat_context;

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
				KERBAL_LOG_WRITE(KVERBOSE, "Get color frame success. camera: {}",
								 camera->device_name()
				);

				cv::Mat const * color_mat = nullptr;
				OBFormat color_format = color_frame->format();
				switch (color_format) {
					case OB_FORMAT_H264: {
						KERBAL_LOG_WRITE(KDEBUG, "Color frame dataSize: {}",
										 color_frame->dataSize());
						color_mat = &H264_to_cv_mat_context.decode(reinterpret_cast<std::uint8_t *>(color_frame->data()), color_frame->dataSize());
						KERBAL_LOG_WRITE(KVERBOSE, "Decode color frame from H.264 success. camera: {}",
										 camera->device_name());
						break;
					}
					case OB_FORMAT_RGB: {
						color_mat = &ob_color_frame_to_cv_mat_context.cast(color_frame);
						KERBAL_LOG_WRITE(KVERBOSE, "Convert ob::Frame to cv::Mat success. camera: {}",
										 camera->device_name());
						break;
					}
					default: {
						KERBAL_LOG_WRITE(KERROR, "Unexpected OB_FORMAT. camera: {}, format: {}",
										 camera->device_name(), color_format);
						throw std::runtime_error("Unexpected OB_FORMAT.");
					}
				}
				if (nullptr == color_mat) {
					KERBAL_LOG_WRITE(KERROR, "color_mat is null. camera: {}",
									 camera->device_name());
					throw std::runtime_error("color_mat is null.");
				}
				try {
					save_cv_mat(*color_mat, filename_rgb);
				} catch (std::exception const & e) {
					KERBAL_LOG_WRITE(KERROR, "Color image saved failed. camera: {}, filename_rgb: {}, exception type: {}, what: {}",
									 camera->device_name(),
									 filename_rgb.string(),
									 typeid(e).name(),
									 e.what()
					);
					throw;
				} catch (...) {
					KERBAL_LOG_WRITE(KERROR, "Color image saved failed. camera: {}, filename_rgb: {}, exception type: unknown",
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
				KERBAL_LOG_WRITE(KVERBOSE, "Get depth frame success. camera: {}, depth_frame.format: {}",
								 camera->device_name(),
								 depth_frame->format()
				);

				std::shared_ptr<ob::Frame> transformed_frame = nullptr;
				int color_height = this->camera->config_rgb.height;
				int color_width = this->camera->config_rgb.width;

				cv::Mat depth_mat;
				if (true) {
					try {
						transformed_frame = this->ob_img_depth_transform_to_color_mode_context.transform(
							this->camera->device(), depth_frame,
							color_height, color_width
						);
					} catch (std::exception const & e) {
						KERBAL_LOG_WRITE(KERROR, "Transforming depth image to color type failed. camera: {}, exception type: {}, what: {}",
										 camera->device_name(),
										 typeid(e).name(),
										 e.what()
						);
						throw;
					} catch (...) {
						KERBAL_LOG_WRITE(KERROR, "Transforming depth image to color type failed. camera: {}, exception type: unknown",
										 filename_depth.string()
						);
						throw;
					}

					KERBAL_LOG_WRITE(KVERBOSE, "Transforming depth image to color type success. camera: {}",
									 camera->device_name()
					);

					depth_mat = ob_frame_depth_to_cv_mat_context.transform(
						transformed_frame,
						color_height, color_width
					);
				} else {
					depth_mat = cv::Mat(depth_frame->height(), depth_frame->width(), CV_16UC1, depth_frame->data());
				}

				try {
					save_cv_mat(depth_mat, filename_depth);
				} catch (std::exception const & e) {
					KERBAL_LOG_WRITE(KERROR, "Depth image saved failed. camera: {}, filename_depth: {}, exception type: {}, what: {}",
									 camera->device_name(),
									 filename_depth.string(),
									 typeid(e).name(),
									 e.what()
					);
					throw;
				} catch (...) {
					KERBAL_LOG_WRITE(KERROR, "Depth image saved failed. camera: {}, filename_depth: {}, exception type: unknown",
									 camera->device_name(),
									 filename_depth.string()
					);
					throw;
				}
			}

	};


	static_assert(dksave::ucamera<camera>, "ob_camera doesn't meet the requirement of ucamera");


} // namespace dksave::plugins_ob

#endif // DKSAVE_PLUGINS_OB_CAMERA_HPP
