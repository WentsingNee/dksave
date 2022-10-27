/**
 * @file       DKSave.cpp
 * @brief
 * @date       2022-10-06
 * @author     Peter
 */

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
#include <shlwapi.h>

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

	bool save_result = imwrite(path.string(), mat);
	if (!save_result) {
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
		cv::Mat & convert(const k4a::image & img)
		{
			// rgb
			// * Each pixel of BGRA32 data is four bytes. The first three bytes represent Blue, Green,
			// * and Red data. The fourth byte is the alpha channel and is unused in the Azure Kinect APIs.

			describe_img(img, "rgb");

			mat_with_alpha = cv::Mat(img.get_height_pixels(), img.get_width_pixels(), CV_8UC4, (void *) (img.get_buffer()));

			cv::cvtColor(mat_with_alpha, mat_no_alpha, cv::COLOR_BGRA2BGR);

			return mat_no_alpha;
		}
};

struct k4a_img_to_depth_rainbow_context_t
{
		cv::Mat mat;
		cv::Mat dst;
		cv::Mat result;

		/**
		 * 将 k4a 深度图像转换为假彩色风格的 cv::Mat
		 * @param img
		 * @return
		 */
		cv::Mat & convert(const k4a::image & img)
		{
			// depth
			// * Each pixel of DEPTH16 data is two bytes of little endian unsigned depth data. The unit of the data is in
			// * millimeters from the origin of the camera.

			describe_img(img, "depth");

			mat = cv::Mat(img.get_height_pixels(), img.get_width_pixels(), CV_16U, (void *) (img.get_buffer()));

			cv::convertScaleAbs(mat, dst, 255 / 65535.0);

			cv::applyColorMap(dst, result, cv::COLORMAP_RAINBOW);
			// 颜色选项可参考：https://learnopencv.com/applycolormap-for-pseudocoloring-in-opencv-c-python/

			return result;
		}

};

struct k4a_img_to_ir_context_t
{
		cv::Mat mat;
		cv::Mat mat_8U;

		cv::Mat & convert(const k4a::image & img)
		{
			// ir
			// * Each pixel of IR16 data is two bytes of little endian unsigned depth data. The value of the data represents
			// * brightness.
			describe_img(img, "ir");

			mat = cv::Mat(img.get_height_pixels(), img.get_width_pixels(), CV_16U, (void *) (img.get_buffer()));

			mat.convertTo(mat_8U, CV_8U);

			return mat_8U;
		}
};

//深度图和RGB图配准
static void do_registration(const DKCamera & camera, const k4a::image & depthImage, const k4a::image & rgbImage, const k4a::image & irImage)
{
	//Get the camera calibration for the entire K4A device, which is used for all transformation functions.
	k4a::calibration k4aCalibration = camera.device.get_calibration(camera.config.depth_mode, camera.config.color_resolution);

	k4a::transformation k4aTransformation = k4a::transformation(k4aCalibration);
	k4a::image transformed_colorImage = k4aTransformation.color_image_to_depth_camera(depthImage, rgbImage);

	cv::Mat cv_rgbImage_with_alpha(transformed_colorImage.get_height_pixels(), transformed_colorImage.get_width_pixels(), CV_8UC4,
								   (void *) transformed_colorImage.get_buffer());

	cv::Mat cv_rgbImage_no_alpha;
	cv::cvtColor(cv_rgbImage_with_alpha, cv_rgbImage_no_alpha, cv::COLOR_BGRA2BGR);

	cv::Mat cv_depth(depthImage.get_height_pixels(), depthImage.get_width_pixels(), CV_16U,
					 (void *) depthImage.get_buffer(), static_cast<size_t>(depthImage.get_stride_bytes()));

	cv::Mat cv_depth_8U;
	normalize(cv_depth, cv_depth_8U, 0, 256 * 256, cv::NORM_MINMAX);
	cv_depth_8U.convertTo(cv_depth, CV_8U, 1);

	cv::Mat cv_irImage(irImage.get_height_pixels(), irImage.get_width_pixels(), CV_16U,
					   (void *) irImage.get_buffer(), static_cast<size_t>(irImage.get_stride_bytes()));

	cv::Mat cv_irImage_8U;
	normalize(cv_irImage, cv_irImage_8U, 0, 256 * 256, cv::NORM_MINMAX);
	cv_irImage.convertTo(cv_irImage_8U, CV_8U, 1);

	cv::imshow("depth", cv_depth_8U);
}


struct depth_img_to_color_context_t
{
		k4a::calibration calibration;
		k4a::transformation transformation;

		depth_img_to_color_context_t(const DKCamera & camera) :
				calibration(
						camera.device.get_calibration(
								camera.config.depth_mode,
								camera.config.color_resolution
						)
				), // Get the camera calibration for the entire K4A device, which is used for all transformation functions.
				transformation(calibration)
		{
		}

		k4a::image color_img;

		k4a::image & convert(const k4a::image & depth_img)
		{
			color_img = transformation.depth_image_to_color_camera(depth_img);
			return color_img;
		}
};

struct k4a_img_to_depth_context_t
{
		cv::Mat cv_depth;

		cv::Mat & convert(const k4a::image & depth_img)
		{
			cv_depth = cv::Mat(depth_img.get_height_pixels(), depth_img.get_width_pixels(), CV_16U,
							   (void *) (depth_img.get_buffer()), static_cast<size_t>(depth_img.get_stride_bytes()));

			return cv_depth;

		}
};


static const std::filesystem::path working_dir = R"(D:\dk.test\)";

/**
 * 一个相机的工作线程
 */
static void camera_working_thread(DKCamera & camera)
{
	camera.stable();
	KERBAL_LOG_WRITE(KINFO, "Camera {} have been stable.", camera.device_id);

	std::filesystem::path camera_working_dir = working_dir / ("camera" + std::to_string(camera.device_id));

	std::filesystem::path path_base_rgb = camera_working_dir / "rgb";
	std::filesystem::path path_base_depth = camera_working_dir / "depth";
	std::filesystem::path path_base_depth_rainbow = camera_working_dir / "rainbow";

	k4a_img_to_rgb_no_alpha_context_t k4a_img_to_rgb_no_alpha_context;
	depth_img_to_color_context_t depth_img_to_color_context(camera);
	k4a_img_to_depth_context_t k4a_img_to_depth_context;
	k4a_img_to_depth_rainbow_context_t k4a_img_to_depth_rainbow_context;

	int count = 0;
	while (true) {

		auto start_time = std::chrono::steady_clock::now();

		SYSTEMTIME time;
		GetLocalTime(&time);

		auto is_night = [&time]() {
			if (time.wHour <= 7) { // 从 8:00:00 开始采到 17:59:59
				return true;
			}
			else if (time.wHour >= 18) {
				return true;
			}
			return false;
		};

		if (is_night()) {
			using namespace std::chrono_literals;
			std::this_thread::sleep_for(1min);
			continue;
		}

		if (!(0 <= time.wMinute && time.wMinute <= 14)) {
			using namespace std::chrono_literals;
			std::this_thread::sleep_for(1s);
			continue;
		}

		try {
			k4a::capture capture;

			try {
				bool capture_result = camera.device.get_capture(&capture);
				if (!capture_result) {
					KERBAL_LOG_WRITE(KERROR, "Camera {}: Get capture failed!", camera.device_id);
					continue;
				}
			} catch (...) {
				KERBAL_LOG_WRITE(KERROR, "Camera {}: Get capture failed!", camera.device_id);
				continue;
			}


			k4a::image rgbImage = capture.get_color_image();

			std::string date = current_date();
			std::string timestamp = current_timestamp();

			const cv::Mat & cv_rgbImage_no_alpha = k4a_img_to_rgb_no_alpha_context.convert(rgbImage);
			std::filesystem::path filename_rgb = path_base_rgb / date / (timestamp + ".png");
			try {
				save_cv_mat(cv_rgbImage_no_alpha, filename_rgb);
			} catch (...) {
				KERBAL_LOG_WRITE(KERROR, "Camera {}: rgb save failed: {}", camera.device_id, filename_rgb.string());
			}

			bool handle_depth = true;
			k4a::image depthImage = capture.get_depth_image();
			const k4a::image * depthImageTran = nullptr;
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

				// 深度图 (假彩色)
//				const cv::Mat & cv_depth_rainbow = k4a_img_to_depth_rainbow_context.apply(*depthImageTran);
//				std::filesystem::path filename_depth_rainbow = path_base_depth_rainbow / date / (timestamp + ".png");
//				try {
//					save_cv_mat(cv_depth_rainbow, filename_depth_rainbow);
//				}
//				catch (...) {
//					KERBAL_LOG_WRITE(KERROR, "Camera {}: rainbow save failed: {}", camera.device_id, filename_depth_rainbow.string());
//				}
			}

			count++;
			KERBAL_LOG_WRITE(KINFO, "Camera {}: Frame {} handle done.", camera.device_id, count);

			cv::waitKey(1);

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


	// 打开设备
	std::vector<DKCamera> cameras;
	cameras.reserve(device_count);

	std::map<uint32_t, k4a_device_configuration_t> configurations = {
			{0, get_config_0()},
			{1, get_config_1()},
	};


	for (uint32_t i = 0; i < device_count; ++i) {

		// 打开设备
		k4a::device device;
		try {
			device = k4a::device::open(i);
		} catch (...) {
			KERBAL_LOG_WRITE(KFATAL, "Camara {} open failed.", i);
			continue;
		}

		auto it = configurations.find(i);
		k4a_device_configuration_t config = (
				it == configurations.cend() ?
				K4A_DEVICE_CONFIG_INIT_DISABLE_ALL :
				it->second
		); // 从 configurations 中查找配置参数, 查不到则使用 K4A_DEVICE_CONFIG_INIT_DISABLE_ALL

		DKCamera & camera = cameras.emplace_back(std::move(device), i, std::move(config), true);

		// 启动设备
		try {
			camera.start();
		} catch (...) {
			camera.enable = false;
			KERBAL_LOG_WRITE(KFATAL, "Camara {} start failed.", camera.device_id);
			continue;
		}

	}


	KERBAL_LOG_WRITE(KINFO, "{} cameras have started.", cameras.size());

	std::vector<std::thread> threads(cameras.size());
	for (size_t i = 0; i < cameras.size(); ++i) {
		if (!cameras[i].enable) {
			continue;
		}
		threads[i] = std::thread(camera_working_thread, std::ref(cameras[i]));
	}

	// 等待所有线程结束
	for (std::thread & thread: threads) {
		thread.join();
	}

	cv::destroyAllWindows();

	KERBAL_LOG_WRITE(KINFO, "Good Bye!");

	return EXIT_SUCCESS;
}
