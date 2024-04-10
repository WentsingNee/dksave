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

		public:
			using super::device_name;
			using super::enable;
			using super::previous_status;

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
					KERBAL_LOG_WRITE(KERROR, "Profile not supported. camera: {}, config: {}, what: {}",
									 this->device_name(), dksave_config, e.getMessage());
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
			std::filesystem::path camera_working_dir;

			std::filesystem::path path_base_color = camera_working_dir / "rgb";
			std::filesystem::path path_base_depth = camera_working_dir / "depth";
			std::filesystem::path path_base_depth_clouds = camera_working_dir / "clouds";

			std::shared_ptr<ob::FrameSet> frame_set;

			struct RGB_frame_to_cv_mat_context
			{
					cv::Mat cv_mat;

					cv::Mat const & cast(std::shared_ptr<ob::ColorFrame> color_frame)
					{
						this->cv_mat = cv::Mat(color_frame->height(), color_frame->width(), CV_8UC3, color_frame->data());
						cv::cvtColor(this->cv_mat, this->cv_mat, cv::COLOR_RGB2BGR);
						return this->cv_mat;
					}
			} rgb_context;

			H264_to_cv_mat_context h264_context;

		public:
			int frame_count = 0;

		public:
			capture_loop_context(class camera * camera, std::filesystem::path const & working_dir) :
				camera(camera),
				camera_working_dir(working_dir / this->camera->device_name())
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
			void handle_color(std::string const & date, std::string const & timestamp)
			{
				if (!frame_set) {
					return;
				}
				auto color_frame = frame_set->colorFrame();
				if (color_frame == nullptr) {
					KERBAL_LOG_WRITE(KERROR, "Get color frame failed. camera: {}", camera->device_name());
					return;
				}
				KERBAL_LOG_WRITE(KDEBUG, "Get color frame success. camera: {}, format: {}",
								 camera->device_name(),
								 color_frame->format());

				cv::Mat const * color_mat = nullptr;
				switch (color_frame->format()) {
					case OB_FORMAT_H264: {
						KERBAL_LOG_WRITE(KDEBUG, "Color frame dataSize: {}",
										 color_frame->dataSize());
						color_mat = &h264_context.decode(reinterpret_cast<std::uint8_t *>(color_frame->data()), color_frame->dataSize());
						KERBAL_LOG_WRITE(KDEBUG, "Decode color frame from H.264 success. camera: {}",
										 camera->device_name());
						break;
					}
					case OB_FORMAT_RGB: {
						color_mat = &rgb_context.cast(color_frame);
						KERBAL_LOG_WRITE(KDEBUG, "Convert ob::Frame to cv::Mat success. camera: {}",
										 camera->device_name());
						break;
					}
				}
				if (nullptr == color_mat) {
					KERBAL_LOG_WRITE(KDEBUG, "color_mat is null. camera: {}",
									 camera->device_name());
					return;
				}
				std::filesystem::path filename_rgb = path_base_color / date / (timestamp + ".png");
				try {
					save_cv_mat(*color_mat, filename_rgb);
				} catch (...) {
					KERBAL_LOG_WRITE(KERROR, "Color image saved failed. camera: {}, filename_rgb: {}",
									 camera->device_name(),
									 filename_rgb.string());
				}
			}

			void handle_depth(std::string const & date, std::string const & timestamp)
			{
				if (!frame_set) {
					return;
				}
				auto depth_frame = frame_set->depthFrame();
				if (depth_frame == nullptr) {
					KERBAL_LOG_WRITE(KERROR, "Get depth frame failed. camera: {}", camera->device_name());
					return;
				}
				KERBAL_LOG_WRITE(KDEBUG, "Get depth frame success. camera: {}, format: {}",
								 camera->device_name(),
								 depth_frame->format());

				cv::Mat depth_mat(depth_frame->height(), depth_frame->width(), CV_16UC1, depth_frame->data());
				std::filesystem::path filename_depth = path_base_depth / date / (timestamp + ".png");
				try {
					save_cv_mat(depth_mat, filename_depth);
				} catch (...) {
					KERBAL_LOG_WRITE(KERROR, "Depth image saved failed. camera: {}, filename_depth: {}",
									 camera->device_name(),
									 filename_depth.string());
				}
			}

	};


	static_assert(dksave::ucamera<camera>, "ob_camera doesn't meet the requirement of ucamera");


} // namespace dksave::plugins_ob

#endif // DKSAVE_PLUGINS_OB_CAMERA_HPP
