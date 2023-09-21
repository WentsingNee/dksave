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
#include <regex>
#include <ctime>
#include <windows.h>

#include "dkconfig.hpp"

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


static std::filesystem::path working_dir = R"(D:\dk.test\)";

using namespace std::chrono_literals;

static std::chrono::seconds start_time = 0h;
static std::chrono::seconds end_time = 23h + 59min + 59s;
static auto prepare_time = 1min;

enum class working_status
{
		SLEEP,
		READY,
		WORK,
};

template<>
class fmt::formatter<working_status>
{
		static char const * to_str(working_status status)
		{
			switch (status) {
				case working_status::SLEEP: {
					return "SLEEP";
				}
				case working_status::READY: {
					return "READY";
				}
				case working_status::WORK: {
					return "WORK";
				}
				default: {
					return "UNKNOWN";
				}
			}
		}

	public:

		auto format(working_status status, format_context & ctx) const -> format_context::iterator
		{

			fmt::format_to(
					ctx.out(),
					"{}", to_str(status)
			);

			return ctx.out();
		}

		constexpr auto parse(format_parse_context & ctx) -> format_parse_context::iterator
		{
			return ctx.begin();
		}
};

static working_status get_working_status(std::chrono::time_point<std::chrono::system_clock> now)
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
static void camera_working_thread(DKCamera & camera, std::chrono::milliseconds sleep_period)
{
	std::filesystem::path camera_working_dir = working_dir / camera.device_name();

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
			camera.device().get_calibration(
					camera.config().depth_mode,
					camera.config().color_resolution
			)
	);

	int count = 0;
	working_status previous_status = working_status::WORK;

	while (true) {

		auto start_time = std::chrono::system_clock::now();

		working_status status = get_working_status(start_time);

		if (status != previous_status) {
			KERBAL_LOG_WRITE(KINFO, "Switch status from {} to {}.", previous_status, status);
			previous_status = status;
		}

		if (status == working_status::SLEEP) {
			if (camera.enable()) {
				KERBAL_LOG_WRITE(KINFO, "Camera {} fall in sleep.", camera.device_name());
				camera.stop();
			}
			std::this_thread::sleep_for(1min);
			continue;
		} else {
			if (!camera.enable()) {
				// 启动设备
				KERBAL_LOG_WRITE(KINFO, "Camera {} is sleeping, try to wake up...", camera.device_name());
				try {
					camera.start();
					KERBAL_LOG_WRITE(KINFO, "Camera {} has been started.", camera.device_name());
				} catch (...) {
					KERBAL_LOG_WRITE(KERROR, "Waking up camara {} failed.", camera.device_name());
					std::this_thread::sleep_for(30s);
					continue;
				}
				try {
					camera.stabilize();
					KERBAL_LOG_WRITE(KINFO, "Camera {} has been stable.", camera.device_name());
				} catch (...) {
					KERBAL_LOG_WRITE(KERROR, "Stabilization process of camara {} failed.", camera.device_name());
					std::this_thread::sleep_for(30s);
					continue;
				}
            }
		}


		try {

			bool capture_success = false;
			try {
				capture_success = camera.device().get_capture(&capture);
			} catch (...) {
				KERBAL_LOG_WRITE(KERROR, "Camera {}: Get capture failed!", camera.device_name());
				std::this_thread::sleep_for(500ms);
				continue;
			}

			if (!capture_success) {
				KERBAL_LOG_WRITE(KERROR, "Camera {}: Get capture failed!", camera.device_name());
				std::this_thread::sleep_for(500ms);
				continue;
			}

			SYSTEMTIME time;
			GetLocalTime(&time);

			std::string date = format_systime_to_date(time);
			std::string timestamp = format_systime_to_timestamp(time);

			k4a::image k4a_img_color = capture.get_color_image();

			const cv::Mat & cv_color_img_without_alpha = k4a_img_color_to_cv_mat_context.convert(k4a_img_color);
			std::filesystem::path filename_color = path_base_color / date / (timestamp + ".png");
			try {
				save_cv_mat(cv_color_img_without_alpha, filename_color);
			} catch (...) {
				KERBAL_LOG_WRITE(KERROR, "Camera {}: color image saved failed: {}", camera.device_name(), filename_color.string());
			}

			bool handle_depth = true;
			k4a::image k4a_img_depth = capture.get_depth_image();
			k4a::image * k4a_img_depth_transformed_to_color = nullptr;
			try {
				k4a_img_depth_transformed_to_color = &k4a_img_depth_transform_to_color_mode_context.transform(transformation, k4a_img_depth);
			} catch (...) {
				KERBAL_LOG_WRITE(KERROR, "Camera {}: depth image transformation to color failed", camera.device_name());
				handle_depth = false;
			}

			if (handle_depth) {
				const cv::Mat & cv_depth_img = k4a_img_to_cv_mat_depth_context.convert(*k4a_img_depth_transformed_to_color);
				std::filesystem::path filename_depth = path_base_depth / date / (timestamp + ".png");
				try {
					save_cv_mat(cv_depth_img, filename_depth);
				} catch (...) {
					KERBAL_LOG_WRITE(KERROR, "Camera {}: depth image saved failed: {}", camera.device_name(), filename_depth.string());
				}

//				const k4a::image & k4a_img_point_cloud = k4a_img_depth_transform_to_point_cloud_mode_context.transform(transformation, *k4a_img_depth_transformed_to_color);
//				const pcl::PointCloud<pcl::PointXYZ> & pcl_point_cloud = k4a_img_point_cloud_to_pcl_point_cloud_context.convert(k4a_img_point_cloud);
//				std::filesystem::path filename_clouds = path_base_depth_clouds / date / (timestamp + ".ply");
//				try {
//					std::filesystem::create_directories(filename_clouds.parent_path());
//					pcl::io::savePLYFile(filename_clouds.string(), pcl_point_cloud);
//					KERBAL_LOG_WRITE(KINFO, "Saved {}", filename_clouds.string());
//				} catch (...) {
//					KERBAL_LOG_WRITE(KERROR, "Camera {}: depth clouds saved failed: {}", camera.device_name(), filename_depth.string());
//				}

			}

			count++;
			KERBAL_LOG_WRITE(KINFO, "Camera {}: Frame {} handled done.", camera.device_name(), count);

		} catch (...) {
			KERBAL_LOG_WRITE(KERROR, "Camera {}: Unhandled exception.", camera.device_name());
		}

		using namespace std::chrono_literals;
		std::this_thread::sleep_until(start_time + sleep_period);

	} // while

}


static
std::vector<DKCamera>
find_and_open_cameras(YAML::Node & yaml_config)
{
	// 找到并打开 Azure Kinect 设备
	uint32_t device_count = k4a::device::get_installed_count(); // 发现已连接的设备数
	if (0 == device_count) {
		KERBAL_LOG_WRITE(KFATAL, "No camera found.");
		exit(EXIT_FAILURE);
	}

	KERBAL_LOG_WRITE(KINFO, "Found {} connected cameras.", device_count);


	std::vector<DKCamera> cameras;
	std::set<std::string> device_name_used;
	cameras.reserve(device_count);


	for (uint32_t i = 0; i < device_count; ++i) {

		// 打开设备
		try {
			k4a::device device = k4a::device::open(i);
			std::string serial_num = device.get_serialnum();
			KERBAL_LOG_WRITE(KINFO, "Serial num of camara {} is {}", i, serial_num);

			YAML::Node camera_node = yaml_config["cameras"][serial_num];
			if (!camera_node) {
				KERBAL_LOG_WRITE(KWARNING, "there is no configuration in yaml. serial_num: {}", serial_num);
				continue;
			}

			YAML::Node camera_k4a_config_node = camera_node["k4a_config"];
			if (!camera_k4a_config_node) {
				KERBAL_LOG_WRITE(KWARNING, "there is no k4a_config in yaml. serial_num: {}", serial_num);
				continue;
			}

			k4a_device_configuration_t config = yaml_to_k4a_config(camera_k4a_config_node);
			std::string device_name;
			try {
				device_name = camera_node["device_name"].as<std::string>();
			} catch (std::exception const & e) {
				KERBAL_LOG_WRITE(KFATAL, "Parse device_name failed. exception type: {}, what: {}", typeid(e).name(), e.what());
				throw;
			}

			if (device_name_used.find(device_name) != device_name_used.cend()) {
				KERBAL_LOG_WRITE(KFATAL, "device_name: {} has been occupied.", device_name);
				exit(EXIT_FAILURE);
			}
			device_name_used.insert(device_name);

			KERBAL_LOG_WRITE(KINFO, "Open the {}-th camera with arg: serial_num: {}, config: {}, device_name: {}", i, serial_num, config, device_name);
			cameras.emplace_back(std::move(device), device_name, std::move(config));
		} catch (std::exception const & e) {
			KERBAL_LOG_WRITE(KERROR, "Camara {} open failed. exception type: {}, what: {}", i, typeid(e).name(), e.what());
		} catch (...) {
			KERBAL_LOG_WRITE(KERROR, "Camara {} open failed. what: unknown exception", i);
			continue;
		}
	}

	KERBAL_LOG_WRITE(KINFO, "{} cameras have been opened.", cameras.size());

	return cameras;
}


/**
 * 将 hh:mm:ss 格式的字符串转换为 std::chrono::seconds
 */
static std::chrono::seconds parse_clock(std::string const & s)
{
	static std::regex const pattern(R"(\d\d:\d\d:\d\d)");
	if (!std::regex_match(s, pattern)) {
		throw std::runtime_error("wrong clock format");
	}

	int hour = std::stoi(s.substr(0, 2));
	int min = std::stoi(s.substr(3, 2));
	int sec = std::stoi(s.substr(6, 2));
	return std::chrono::hours(hour) + std::chrono::minutes(min) + std::chrono::seconds(sec);
}


int main(int argc, char * argv[]) try
{
	if (argc < 2) {
		std::cerr << fmt::format("Usage: {} dkconfig.yaml", argv[0]) << std::endl;
		exit(EXIT_FAILURE);
	}

	std::filesystem::path dkconfig_yaml_path = argv[1];

	KERBAL_LOG_WRITE(KINFO, "Config YAML path: {}", dkconfig_yaml_path.string());
	YAML::Node yaml_root_node;
	try {
		yaml_root_node = YAML::LoadFile(dkconfig_yaml_path.string());
	} catch (YAML::BadFile const & e) {
		KERBAL_LOG_WRITE(KFATAL, "DK config yaml cannot be loaded. what: {}", e.what());
		throw;
	} catch (YAML::ParserException const & e) {
		KERBAL_LOG_WRITE(KFATAL, "DK config yaml has format error. what: {}", e.what());
		throw;
	}

    YAML::Node config_node = yaml_root_node["config"];

	try {
		std::string working_dir_str;
		working_dir_str = config_node["working_dir"].as<std::string>();
		working_dir = std::filesystem::path(working_dir_str);
		KERBAL_LOG_WRITE(KINFO, "Parse sleep_period success. working_dir: {}", working_dir_str);
	} catch (YAML::InvalidNode const & e) {
		KERBAL_LOG_WRITE(KFATAL, "Parse sleep_period failed. what: {}", e.what());
		throw;
	}


	std::chrono::milliseconds sleep_period;
	try {
		int sleep_period_i = config_node["sleep_period"].as<int>();
		sleep_period = std::chrono::milliseconds(sleep_period_i);
		KERBAL_LOG_WRITE(KINFO, "Parse sleep_period success. sleep_period: {} ms", sleep_period.count());
	} catch (YAML::InvalidNode const & e) {
		KERBAL_LOG_WRITE(KFATAL, "Parse sleep_period failed. what: {}", e.what());
		throw;
	}


	try {
		std::string s = config_node["start_time"].as<std::string>();
		start_time = parse_clock(s);
		KERBAL_LOG_WRITE(KINFO, "Parse start_time success. start_time: {}", s);
	} catch (YAML::InvalidNode const & e) {
		KERBAL_LOG_WRITE(KFATAL, "Parse start_time failed. what: {}", e.what());
		throw;
	} catch (std::exception const & e) {
		KERBAL_LOG_WRITE(KFATAL, "Parse start_time failed. what: {}", e.what());
		throw;
	}


	try {
		std::string s = config_node["end_time"].as<std::string>();
		end_time = parse_clock(s);
		KERBAL_LOG_WRITE(KINFO, "Parse end_time success. end_time: {}", s);
	} catch (YAML::InvalidNode const & e) {
		KERBAL_LOG_WRITE(KFATAL, "Parse end_time failed. what: {}", e.what());
		throw;
	} catch (std::exception const & e) {
		KERBAL_LOG_WRITE(KFATAL, "Parse end_time failed. what: {}", e.what());
		throw;
	}



	std::vector<DKCamera> cameras = find_and_open_cameras(config_node);

	std::vector<std::thread> threads;
	threads.reserve(cameras.size());
	for (DKCamera & camera: cameras) {
		threads.emplace_back(camera_working_thread, std::ref(camera), sleep_period);
	}

	// 等待所有线程结束
	for (std::thread & thread: threads) {
		thread.join();
	}

	KERBAL_LOG_WRITE(KINFO, "Good Bye!");

	return EXIT_SUCCESS;
} catch (std::exception const & e) {
	KERBAL_LOG_WRITE(KFATAL, "DK Save exit with exception. exception type: {}, what: {}", typeid(e).name(), e.what());
} catch (...) {
	KERBAL_LOG_WRITE(KFATAL, "DK Save exit with exception. exception type: unknown");
}
