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

#include "ucamera.hpp"

#include <memory>

#include <libobsensor/ObSensor.hpp>
#include <opencv2/core.hpp>


class ob_camera : public ucamera {
		using super = ucamera;

		std::shared_ptr<ob::Device> k_device;

	public:
		ob_camera(
				std::shared_ptr<ob::Device> &&device,
				std::string const &device_name) :
				super(device_name),
				k_device(std::move(device)) {
		}

		std::shared_ptr<ob::Device> &device() {
			return k_device;
		}

		virtual void start() override {

		}

		virtual void stabilize() override {

		}

		virtual void stop() noexcept override {

		}

		virtual void
		working_loop(std::filesystem::path const &working_dir, std::chrono::milliseconds sleep_period) override {
			std::filesystem::path camera_working_dir = working_dir / this->device_name();

			std::filesystem::path path_base_color = camera_working_dir / "rgb";
			std::filesystem::path path_base_depth = camera_working_dir / "depth";
			std::filesystem::path path_base_depth_clouds = camera_working_dir / "clouds";

			ob::Pipeline pipeline(this->k_device);

			while (true) {
				auto frame_set = pipeline.waitForFrames(sleep_period.count());

				SYSTEMTIME time;
				GetLocalTime(&time);

				std::string date = format_systime_to_date(time);
				std::string timestamp = format_systime_to_timestamp(time);

				auto color_frame = frame_set->colorFrame();

				cv::Mat color_mat(color_frame->height(), color_frame->width(), CV_8UC4, color_frame->data());

			}
		}
};

#endif // DKSAVE_OB_CAMERA_HPP
