/**
 * @file       DKSave.cpp
 * @brief
 * @date       2022-10-06
 * @author     Peter
 */

#include <chrono>
#include <string>
#include <vector>
#include <thread>
#include <map>
#include <filesystem>
#include <ctime>

// OpenCV
#include <opencv2/opencv.hpp>

// Kinect DK
#include <k4a/k4a.hpp>
#include <windows.h>

#include "logger.hpp"
#include "DKCamera.hpp"


static void describe_img(const k4a::image & img, const char type[])
{
	KERBAL_LOG_WRITE(KDEBUG, "[{}]", type);
	KERBAL_LOG_WRITE(KDEBUG, "format: {}", img.get_format());
	KERBAL_LOG_WRITE(KDEBUG, "device_timestamp: {}", img.get_device_timestamp().count());
	KERBAL_LOG_WRITE(KDEBUG, "system_timestamp: {}", img.get_system_timestamp().count());
	KERBAL_LOG_WRITE(KDEBUG, "height: {}, width: {}", img.get_height_pixels(), img.get_width_pixels());
}

static void save_cv_mat(const cv::Mat & mat, const std::filesystem::path & path)
{
	std::filesystem::create_directories(path.parent_path());

	bool write_success = imwrite(path.string(), mat);
	if (!write_success) {
		throw std::runtime_error("Save failed.");
	}

	KERBAL_LOG_WRITE(KINFO, "Saved {}", path.string());
}

struct k4a_img_to_rgb_no_alpha_context_t
{
		cv::Mat mat_with_alpha;
		cv::Mat mat_no_alpha;

		/**
		 * 将 k4a 可见光图像转换为无透明度通道的 cv::Mat
		 * @param img 一般为通过 capture.get_color_image() 得到的图像
		 * @return
		 */
		cv::Mat & convert(k4a::image & img)
		{
			// rgb
			// * Each pixel of BGRA32 data is four bytes. The first three bytes represent Blue, Green,
			// * and Red data. The fourth byte is the alpha channel and is unused in the Azure Kinect APIs.

			describe_img(img, "rgb");

			mat_with_alpha = cv::Mat(img.get_height_pixels(), img.get_width_pixels(), CV_8UC4, static_cast<void *>(img.get_buffer()));

			cv::cvtColor(mat_with_alpha, mat_no_alpha, cv::COLOR_BGRA2BGR);

			return mat_no_alpha;
		}
};

struct k4a_img_to_ir_context_t
{
		cv::Mat mat;
		cv::Mat mat_8U;

		cv::Mat & convert(k4a::image & img)
		{
			// ir
			// * Each pixel of IR16 data is two bytes of little endian unsigned depth data. The value of the data represents
			// * brightness.
			describe_img(img, "ir");

			mat = cv::Mat(img.get_height_pixels(), img.get_width_pixels(), CV_16U, static_cast<void *>(img.get_buffer()));

			mat.convertTo(mat_8U, CV_8U);

			return mat_8U;
		}
};


struct depth_img_to_color_context_t
{
		k4a::calibration calibration;
		k4a::transformation transformation;

		depth_img_to_color_context_t(const DKCamera & camera) :
				// Get the camera calibration for the entire K4A device, which is used for all transformation functions.
				calibration(
					camera.device.get_calibration(
						camera.config.depth_mode,
						camera.config.color_resolution
					)
				),
				transformation(calibration)
		{
		}

		k4a::image depth_img_regi_to_rgb; // 深度向 rgb 配准的图像

		k4a::image & convert(const k4a::image & depth_img)
		{
			depth_img_regi_to_rgb = transformation.depth_image_to_color_camera(depth_img);
			return depth_img_regi_to_rgb;
		}
};

struct k4a_img_to_depth_context_t
{
		cv::Mat cv_depth;

		cv::Mat & convert(k4a::image & depth_img)
		{
			cv_depth = cv::Mat(depth_img.get_height_pixels(), depth_img.get_width_pixels(), CV_16U,
							   static_cast<void *>(depth_img.get_buffer()), static_cast<size_t>(depth_img.get_stride_bytes()));

			return cv_depth;
		}
};


static const std::filesystem::path working_dir = R"(D:\dk.test\)";

using namespace std::chrono_literals;

auto start_time = 8h;
auto end_time = 18h;
auto prepare_time = 1min;

enum class working_status
{
	SLEEP,
	READY,
	WORK,
};

static working_status judge(std::chrono::time_point<std::chrono::system_clock> now)
{
	const std::chrono::time_zone * tz = std::chrono::current_zone();
	auto local_now = tz->to_local(now);
	auto today_midnight = std::chrono::floor<std::chrono::days>(local_now);
	auto time_since_midnight = local_now - today_midnight;

	if (start_time <= time_since_midnight && time_since_midnight < end_time) {
		return working_status::WORK;
	}
	auto t = (time_since_midnight + prepare_time) % 24h;
	if (start_time <= t && t < end_time) {
		return working_status::READY;
	}
	return working_status::SLEEP;
}


/**
 * 一个相机的工作线程
 */
static void camera_working_thread(DKCamera & camera)
{
	std::filesystem::path camera_working_dir = working_dir / ("camera" + std::to_string(camera.device_id));

	std::filesystem::path path_base_rgb = camera_working_dir / "rgb";
	std::filesystem::path path_base_depth = camera_working_dir / "depth";
	std::filesystem::path path_base_depth_rainbow = camera_working_dir / "rainbow";
	
	k4a::capture capture;
	k4a_img_to_rgb_no_alpha_context_t k4a_img_to_rgb_no_alpha_context;
	depth_img_to_color_context_t depth_img_to_color_context(camera);
	k4a_img_to_depth_context_t k4a_img_to_depth_context;

	int count = 0;
	while (true) {

		auto start_time = std::chrono::system_clock::now();

		working_status status = judge(start_time);
		if (status == working_status::SLEEP) {
			if (camera.enable) {
				KERBAL_LOG_WRITE(KINFO, "Camera {} fall in sleep.", camera.device_id);
				camera.stop();
			}
			std::this_thread::sleep_for(1min);
			continue;
		} else {
			if (!camera.enable) {
				// 启动设备
				KERBAL_LOG_WRITE(KINFO, "Camera {} is sleeping, try to wake up...", camera.device_id);
				try {
					camera.start();
					KERBAL_LOG_WRITE(KINFO, "Camera {} has been started.", camera.device_id);
				} catch (...) {
					camera.enable = false;
					KERBAL_LOG_WRITE(KERROR, "Waking up camara {} failed.", camera.device_id);
					std::this_thread::sleep_for(30s);
					continue;
				}
				try {
					camera.stabilize();
					KERBAL_LOG_WRITE(KINFO, "Camera {} has been stable.", camera.device_id);
				} catch (...) {
					camera.enable = false;
					KERBAL_LOG_WRITE(KERROR, "Stabilization process of camara {} failed.", camera.device_id);
					std::this_thread::sleep_for(30s);
					continue;
				}
			}
		}

		
		SYSTEMTIME time;
		GetLocalTime(&time);
		if (!(0 <= time.wMinute && time.wMinute <= 14)) {
			using namespace std::chrono_literals;
			std::this_thread::sleep_for(1s);
			continue;
		}

		try {

			bool capture_success = true;
			try {
				capture_success = camera.device.get_capture(&capture);
			} catch (...) {
				KERBAL_LOG_WRITE(KERROR, "Camera {}: Get capture failed!", camera.device_id);
				std::this_thread::sleep_until(start_time + 500ms);
				continue;
			}

			if (!capture_success) {
				KERBAL_LOG_WRITE(KERROR, "Camera {}: Get capture failed!", camera.device_id);
				std::this_thread::sleep_until(start_time + 500ms);
				continue;
			}

			
			std::string date = current_date();
			std::string timestamp = current_timestamp();


			k4a::image rgbImage = capture.get_color_image();

			const cv::Mat & cv_rgbImage_no_alpha = k4a_img_to_rgb_no_alpha_context.convert(rgbImage);
			std::filesystem::path filename_rgb = path_base_rgb / date / (timestamp + ".png");
			try {
				save_cv_mat(cv_rgbImage_no_alpha, filename_rgb);
			} catch (...) {
				KERBAL_LOG_WRITE(KERROR, "Camera {}: rgb save failed: {}", camera.device_id, filename_rgb.string());
			}

			bool handle_depth = true;
			k4a::image depthImage = capture.get_depth_image();
			k4a::image * depthImageTran = nullptr;
			try {
				depthImageTran = &depth_img_to_color_context.convert(depthImage);
			} catch (...) {
				KERBAL_LOG_WRITE(KERROR, "Camera {}: depth save transform failed", camera.device_id);
				handle_depth = false;
			}

			if (handle_depth) {
				const cv::Mat & cv_depth = k4a_img_to_depth_context.convert(*depthImageTran);
				std::filesystem::path filename_depth = path_base_depth / date / (timestamp + ".png");
				try {
					save_cv_mat(cv_depth, filename_depth);
				} catch (...) {
					KERBAL_LOG_WRITE(KERROR, "Camera {}: depth save failed: {}", camera.device_id, filename_depth.string());
				}
			}

			count++;
			KERBAL_LOG_WRITE(KINFO, "Camera {}: Frame {} handle done.", camera.device_id, count);

		} catch (...) {
			KERBAL_LOG_WRITE(KERROR, "Camera {}: Unhandled exception.", camera.device_id);
		}

		using namespace std::chrono_literals;
		std::this_thread::sleep_until(start_time + 480ms);

	} // while

}



int main(int argc, char * argv[])
{
	// 找到并打开 Azure Kinect 设备
	uint32_t device_count = k4a::device::get_installed_count(); // 发现已连接的设备数
	if (0 == device_count) {
		KERBAL_LOG_WRITE(KFATAL, "No camera found.");
		exit(EXIT_FAILURE);
	}

	KERBAL_LOG_WRITE(KINFO, "Found {} connected cameras.", device_count);


	std::vector<DKCamera> cameras;
	cameras.reserve(device_count);

	std::map<std::string, std::tuple<int, k4a_device_configuration_t>> configurations = {
			{"000642213912", {0, get_config_0()}},
			{"001376414312", {1, get_config_1()}},
			{"000574514912", {2, get_config_1()}},
	};


	for (uint32_t i = 0; i < device_count; ++i) {

		// 打开设备
		try {
			k4a::device device = k4a::device::open(i);
			std::string serialnum = device.get_serialnum();
			KERBAL_LOG_WRITE(KINFO, "Serial num of camara {} is {}", i, serialnum);

			auto it = configurations.find(serialnum);
			k4a_device_configuration_t config;
			int device_id;
			if (it == configurations.cend()) { // 从 configurations 中查找配置参数, 查不到则使用 K4A_DEVICE_CONFIG_INIT_DISABLE_ALL
				device_id = 9999;
				config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
			} else {
				std::tie(device_id, config) = it->second;
			}

			cameras.emplace_back(std::move(device), device_id, std::move(config));
		} catch (...) {
			KERBAL_LOG_WRITE(KERROR, "Camara {} open failed.", i);
			continue;
		}
	}

	KERBAL_LOG_WRITE(KINFO, "{} cameras have been opened.", cameras.size());

	std::vector<std::thread> threads;
	threads.reserve(cameras.size());
	for (DKCamera & camera : cameras) {
		threads.emplace_back(camera_working_thread, std::ref(camera));
	}

	// 等待所有线程结束
	for (std::thread & thread: threads) {
		thread.join();
	}

	KERBAL_LOG_WRITE(KINFO, "Good Bye!");

	return EXIT_SUCCESS;
}
