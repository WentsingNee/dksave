/**
 * @file       ob_camera.hpp
 * @brief
 * @date       2023-09-24
 * @author     Peter
 * @copyright
 *      Peter of [ThinkSpirit Laboratory](http://thinkspirit.org/)
 *   of [Nanjing University of Information Science & Technology](http://www.nuist.edu.cn/)
 *   all rights reserved
 */


#ifndef DKSAVE_OB_CAMERA_HPP
#define DKSAVE_OB_CAMERA_HPP

#include "config/ob_config.hpp"
#include "logger.hpp"
#include "save_cv_mat.hpp"
#include "ucamera.hpp"

#include <memory>
#include <string>
#include <filesystem>

#include <libobsensor/ObSensor.hpp>
#include <opencv2/core.hpp>


namespace dksave_ob {

	class capture_loop_context;

	class camera : private ucamera_base {
			using super = ucamera_base;

			std::shared_ptr<ob::Device> k_device;
			std::shared_ptr<ob::Config> config;
			std::shared_ptr<ob::Pipeline> pipeline;

		public:
			using super::device_name;
			using super::enable;
			using super::previous_status;

		private:
			static void show_profiles_supported(std::shared_ptr<ob::StreamProfileList> profile_list) {
				for (uint32_t i = 0; i < profile_list->count(); ++i) {
					auto profile = std::const_pointer_cast<ob::StreamProfile>(
							profile_list->getProfile(i))->as<ob::VideoStreamProfile>();
					if (profile) {
						std::cout << fmt::format("{}", *profile) << std::endl;
					} else {
						std::cout << fmt::format("nil") << std::endl;
					}
				}
			}

			void enable_color_stream(std::shared_ptr<ob::Config> config) {
				KERBAL_LOG_WRITE(KINFO, "Enabling camera's color stream. camera: {}", this->device_name());
				auto profile_list = pipeline->getStreamProfileList(OB_SENSOR_COLOR);
				show_profiles_supported(profile_list);
				std::shared_ptr<ob::VideoStreamProfile> profile = nullptr;
				try {
					profile = profile_list->getVideoStreamProfile(1920, 1080, OB_FORMAT_RGB, 30);
				} catch (ob::Error const &e) {
					KERBAL_LOG_WRITE(KERROR, "Profile not supported. camera: {}, what: {}", this->device_name(),
									 e.getMessage());
				}
				if (profile) {
					config->enableStream(profile);
					KERBAL_LOG_WRITE(KINFO, "Enable color stream with profile: {}", *profile);
				}
			}

			void enable_depth_stream(std::shared_ptr<ob::Config> config) {
				KERBAL_LOG_WRITE(KINFO, "Enabling camera's depth stream. camera: {}", this->device_name());
				auto profile_list = pipeline->getStreamProfileList(OB_SENSOR_DEPTH);
				show_profiles_supported(profile_list);
				std::shared_ptr<ob::VideoStreamProfile> profile = nullptr;
				try {
					profile = profile_list->getVideoStreamProfile(640, 576, OB_FORMAT_Y16, 30);
				} catch (ob::Error const &e) {
					KERBAL_LOG_WRITE(KERROR, "Profile not supported. camera: {}, what: {}", this->device_name(),
									 e.getMessage());
				}
				if (profile) {
					config->enableStream(profile);
					KERBAL_LOG_WRITE(KINFO, "Enable depth stream with profile: {}", *profile);
				}
			}

		public:
			camera(
					std::shared_ptr<ob::Device> &&device,
					std::string const &device_name) :
					super(device_name),
					k_device(std::move(device)),
					config(std::make_shared<ob::Config>()),
					pipeline(std::make_shared<ob::Pipeline>(this->device()))
			{
				KERBAL_LOG_WRITE(KINFO, "Creating camera. camera: {}", this->device_name());
				this->enable_color_stream(config);
				this->enable_depth_stream(config);
				config->setAlignMode(ALIGN_D2C_HW_MODE);
				config->setDepthScaleRequire(true);
			}

			std::shared_ptr<ob::Device> &device() {
				return k_device;
			}

			void start() {
				pipeline->start(config);
			}

			void stabilize() {

			}

			void stop() noexcept {
				pipeline->stop();
			}

			using capture_loop_context = dksave_ob::capture_loop_context;
			friend capture_loop_context;
	};

	static_assert(::ucamera<camera>, "ob_camera doesn't meet the requirement of ucamera");


	class capture_loop_context {
			camera *camera;

			std::filesystem::path camera_working_dir;

			std::filesystem::path path_base_color = camera_working_dir / "rgb";
			std::filesystem::path path_base_depth = camera_working_dir / "depth";
			std::filesystem::path path_base_depth_clouds = camera_working_dir / "clouds";

			std::shared_ptr<ob::FrameSet> frame_set;

		public:
			int frame_count = 0;

		public:
			capture_loop_context(class camera *camera, std::filesystem::path const &working_dir) :
					camera(camera),
					camera_working_dir(working_dir / this->camera->device_name()) {
			}

			void do_capture() {
				frame_set = camera->pipeline->waitForFrames(2000);
				if (frame_set == nullptr) {
					KERBAL_LOG_WRITE(KERROR, "Get capture failed. camera: {}", camera->device_name());
					throw std::runtime_error("Get capture failed");
				}
			}

		private:
			static inline std::vector<int> const compression_params = {
					cv::IMWRITE_PNG_COMPRESSION,
					0,
					cv::IMWRITE_PNG_STRATEGY,
					cv::IMWRITE_PNG_STRATEGY_DEFAULT
			};

		public:
			void handle_color(std::string const &date, std::string const &timestamp) {
				if (!frame_set) {
					return;
				}
				auto color_frame = frame_set->colorFrame();
				if (color_frame == nullptr) {
					KERBAL_LOG_WRITE(KERROR, "Get color frame failed. camera: {}", camera->device_name());
					return;
				}

				cv::Mat color_mat(color_frame->height(), color_frame->width(), CV_8UC3, color_frame->data());
				std::filesystem::path filename_rgb = path_base_color / date / (timestamp + ".png");
				try {
					save_cv_mat(color_mat, filename_rgb);
				} catch (...) {
					KERBAL_LOG_WRITE(KERROR, "Color image saved failed. camera: {}, filename_rgb: {}",
									 camera->device_name(),
									 filename_rgb.string());
				}
			}

			void handle_depth(std::string const &date, std::string const &timestamp) {
				if (!frame_set) {
					return;
				}
				auto depth_frame = frame_set->depthFrame();
				if (depth_frame == nullptr) {
					KERBAL_LOG_WRITE(KERROR, "Get depth frame failed. camera: {}", camera->device_name());
					return;
				}

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

} // namespace dksave_ob

#endif // DKSAVE_OB_CAMERA_HPP
