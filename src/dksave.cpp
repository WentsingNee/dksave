/**
 * @file       DKSave.cpp
 * @brief
 * @date       2022-10-06
 * @author     Peter
 */

#include "logger.hpp"
#include "DKCamera.hpp"

#include "context/k4a_img_color_to_cv_mat_context_t.hpp"
#include "context/k4a_img_depth_transform_to_color_mode_context_t.hpp"
#include "context/k4a_img_depth_to_cv_mat_context_t.hpp"
#include "context/k4a_img_depth_transform_to_point_cloud_mode_context_t.hpp"
#include "context/k4a_img_point_cloud_to_pcl_point_cloud_context_t.hpp"

#include <chrono>
#include <string>
#include <vector>
#include <thread>
#include <map>
#include <filesystem>
#include <ctime>
#include <windows.h>


// OpenCV
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

// Kinect DK
#include <k4a/k4a.hpp>

// PCL
#include <pcl/io/ply_io.h>


static void describe_img(const k4a::image & img, const char type[])
{
	KERBAL_LOG_WRITE(KDEBUG, "[{}]", type);
	KERBAL_LOG_WRITE(KDEBUG, "format: {}", img.get_format());
	KERBAL_LOG_WRITE(KDEBUG, "device_timestamp: {}", img.get_device_timestamp().count());
	KERBAL_LOG_WRITE(KDEBUG, "system_timestamp: {}", img.get_system_timestamp().count());
	KERBAL_LOG_WRITE(KDEBUG, "height: {}, width: {}", img.get_height_pixels(), img.get_width_pixels());
}


/**
 * 将一个 cv::Mat 矩阵保存在指定路径. 若指定路径的上层文件夹不存在, 则会自动创建
 */
static void save_cv_mat(const cv::Mat & mat, const std::filesystem::path & path)
{
	std::filesystem::create_directories(path.parent_path());

	bool write_success = imwrite(path.string(), mat);
	if (!write_success) {
		throw std::runtime_error("Save failed.");
	}

	KERBAL_LOG_WRITE(KINFO, "Saved {}", path.string());
}


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
	return working_status::WORK;
//	const std::chrono::time_zone * tz = std::chrono::current_zone();
//	auto local_now = tz->to_local(now);
//	auto today_midnight = std::chrono::floor<std::chrono::days>(local_now);
//	auto time_since_midnight = local_now - today_midnight;
//
//	if (start_time <= time_since_midnight && time_since_midnight < end_time) {
//		auto min = time_since_midnight % 60min;
//		if (0min <= min && min <= 15min) {
//			return working_status::WORK;
//		}
//		if (59min <= min) {
//			return working_status::READY;
//		}
//		return working_status::SLEEP;
//	}
//	auto t = (time_since_midnight + prepare_time) % 24h;
//	if (start_time <= t && t < end_time) {
//		return working_status::READY;
//	}
//	return working_status::SLEEP;
}


/**
 * 一个相机的工作线程
 */
static void camera_working_thread(DKCamera & camera)
{
	std::filesystem::path camera_working_dir = working_dir / ("camera" + std::to_string(camera.device_id));

	std::filesystem::path path_base_color = camera_working_dir / "rgb";
	std::filesystem::path path_base_depth = camera_working_dir / "depth";
	std::filesystem::path path_base_depth_clouds = camera_working_dir / "clouds";

	k4a_img_color_to_cv_mat_context_t k4a_img_color_to_cv_mat_context;
	k4a_img_depth_transform_to_color_mode_context_t k4a_img_depth_transform_to_color_mode_context;
	k4a_img_depth_to_cv_mat_context_t k4a_img_to_cv_mat_depth_context;
	k4a_img_depth_transform_to_point_cloud_mode_context_t k4a_img_depth_transform_to_point_cloud_mode_context;
	k4a_img_point_cloud_to_pcl_point_cloud_context_t k4a_img_point_cloud_to_pcl_point_cloud_context;

	k4a::capture capture;
	k4a::transformation transformation(
			camera.device.get_calibration(
					camera.config.depth_mode,
					camera.config.color_resolution
			)
	);

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


		try {

			bool capture_success = true;
			try {
				capture_success = camera.device.get_capture(&capture);
			} catch (...) {
				KERBAL_LOG_WRITE(KERROR, "Camera {}: Get capture failed!", camera.device_id);
				std::this_thread::sleep_for(500ms);
				continue;
			}

			if (!capture_success) {
				KERBAL_LOG_WRITE(KERROR, "Camera {}: Get capture failed!", camera.device_id);
				std::this_thread::sleep_for(500ms);
				continue;
			}

			SYSTEMTIME time;
			GetLocalTime(&time);
//			if (!(0 <= time.wMinute && time.wMinute <= 14)) {
//				using namespace std::chrono_literals;
//				std::this_thread::sleep_for(1s);
//				continue;
//			}

			std::string date = format_systime_to_date(time);
			std::string timestamp = format_systime_to_timestamp(time);

			k4a::image k4a_img_color = capture.get_color_image();

			const cv::Mat & cv_color_img_without_alpha = k4a_img_color_to_cv_mat_context.convert(k4a_img_color);
			std::filesystem::path filename_color = path_base_color / date / (timestamp + ".png");
			try {
				save_cv_mat(cv_color_img_without_alpha, filename_color);
			} catch (...) {
				KERBAL_LOG_WRITE(KERROR, "Camera {}: color save failed: {}", camera.device_id, filename_color.string());
			}

			bool handle_depth = true;
			k4a::image k4a_img_depth = capture.get_depth_image();
			k4a::image * k4a_img_depth_transformed_to_color = nullptr;
			try {
				k4a_img_depth_transformed_to_color = &k4a_img_depth_transform_to_color_mode_context.transform(transformation, k4a_img_depth);
			} catch (...) {
				KERBAL_LOG_WRITE(KERROR, "Camera {}: depth save transform failed", camera.device_id);
				handle_depth = false;
			}

			if (handle_depth) {
				const cv::Mat & cv_depth_img = k4a_img_to_cv_mat_depth_context.convert(*k4a_img_depth_transformed_to_color);
				std::filesystem::path filename_depth = path_base_depth / date / (timestamp + ".png");
				try {
					save_cv_mat(cv_depth_img, filename_depth);
				} catch (...) {
					KERBAL_LOG_WRITE(KERROR, "Camera {}: depth save failed: {}", camera.device_id, filename_depth.string());
				}

				const k4a::image & k4a_img_point_cloud = k4a_img_depth_transform_to_point_cloud_mode_context.transform(transformation, *k4a_img_depth_transformed_to_color);
				const pcl::PointCloud<pcl::PointXYZ> & pcl_point_cloud = k4a_img_point_cloud_to_pcl_point_cloud_context.convert(k4a_img_point_cloud);
				std::filesystem::path filename_clouds = path_base_depth_clouds / date / (timestamp + ".ply");
				try {
					std::filesystem::create_directories(filename_clouds.parent_path());
					pcl::io::savePLYFile(filename_clouds.string(), pcl_point_cloud);
					KERBAL_LOG_WRITE(KINFO, "Saved {}", filename_clouds.string());
				} catch (...) {
					KERBAL_LOG_WRITE(KERROR, "Camera {}: depth clouds save failed: {}", camera.device_id, filename_depth.string());
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


static
std::vector<DKCamera>
find_and_open_cameras()
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
			std::string serial_num = device.get_serialnum();
			KERBAL_LOG_WRITE(KINFO, "Serial num of camara {} is {}", i, serial_num);

			auto it = configurations.find(serial_num);
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

	return cameras;
}



int main(int argc, char * argv[])
{
	std::vector<DKCamera> cameras =	find_and_open_cameras();

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
