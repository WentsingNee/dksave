/**
 * @file       DKSave.cpp
 * @brief
 * @date       2022-10-06
 * @author     Peter
 */

#include <iostream>
#include <vector>
#include <thread>
#include <ctime>
#include <filesystem>

// OpenCV
#include <opencv2/opencv.hpp>

// Kinect DK
#include <k4a/k4a.hpp>
#include <windows.h>
#include <shlwapi.h>

#include "logger.hpp"
#include "DKCamera.hpp"


static void destribe_image(const k4a::image & img, const char type[])
{
	KERBAL_LOG_WRITE(KDEBUG, "[{}]", type);
	KERBAL_LOG_WRITE(KDEBUG, "format: {}", img.get_format());
	KERBAL_LOG_WRITE(KDEBUG, "device_timestamp: {}", img.get_device_timestamp().count());
	KERBAL_LOG_WRITE(KDEBUG, "system_timestamp: {}", img.get_system_timestamp().count());
	KERBAL_LOG_WRITE(KDEBUG, "height: {}, width: {}", img.get_height_pixels(), img.get_width_pixels());
}

static cv::Mat rgb(const k4a::capture & capture)
{
	// rgb
	// * Each pixel of BGRA32 data is four bytes. The first three bytes represent Blue, Green,
	// * and Red data. The fourth byte is the alpha channel and is unused in the Azure Kinect APIs.

	k4a::image rgbImage = capture.get_color_image();
	destribe_image(rgbImage, "rgb");

	cv::Mat cv_rgbImage_with_alpha(rgbImage.get_height_pixels(), rgbImage.get_width_pixels(), CV_8UC4, (void *) rgbImage.get_buffer());

	cv::Mat cv_rgbImage_no_alpha;
	cv::cvtColor(cv_rgbImage_with_alpha, cv_rgbImage_no_alpha, cv::COLOR_BGRA2BGR);

	return cv_rgbImage_no_alpha;
}

static cv::Mat depth(const k4a::capture & capture)
{
	// depth
	// * Each pixel of DEPTH16 data is two bytes of little endian unsigned depth data. The unit of the data is in
	// * millimeters from the origin of the camera.

	k4a::image depthImage = capture.get_depth_image();
	destribe_image(depthImage, "depth");

	cv::Mat cv_depth(depthImage.get_height_pixels(), depthImage.get_width_pixels(), CV_16U, depthImage.get_buffer());
	cv::Mat cv_depth_8U;
	normalize(cv_depth, cv_depth_8U, 0, 256 * 256, cv::NORM_MINMAX);
	cv_depth.convertTo(cv_depth_8U, CV_8U);

	return cv_depth;
}

static void ir(const k4a::capture & capture)
{
	// ir
	// * Each pixel of IR16 data is two bytes of little endian unsigned depth data. The value of the data represents
	// * brightness.
	k4a::image irImage = capture.get_ir_image();
	destribe_image(irImage, "ir");

	cv::Mat cv_irImage(irImage.get_height_pixels(), irImage.get_width_pixels(), CV_16U, (void *) irImage.get_buffer());
	cv::Mat cv_irImage_8U;
	cv_irImage.convertTo(cv_irImage_8U, CV_8U, 1);
}



//深度图和RGB图配准
//			k4a::image transformed_colorImage;
//Get the camera calibration for the entire K4A device, which is used for all transformation functions.
//k4a::calibration k4aCalibration = device.get_calibration(config.depth_mode, config.color_resolution);

//k4a::transformation k4aTransformation = k4a::transformation(k4aCalibration);

//transformed_colorImage = k4aTransformation.color_image_to_depth_camera(depthImage, rgbImage);

//cv_rgbImage_with_alpha = cv::Mat(transformed_colorImage.get_height_pixels(), transformed_colorImage.get_width_pixels(), CV_8UC4,
//	(void*)transformed_colorImage.get_buffer());
//cv::cvtColor(cv_rgbImage_with_alpha, cv_rgbImage_no_alpha, cv::COLOR_BGRA2BGR);

//cv_depth = cv::Mat(depthImage.get_height_pixels(), depthImage.get_width_pixels(), CV_16U,
//	(void*)depthImage.get_buffer(), static_cast<size_t>(depthImage.get_stride_bytes()));

//normalize(cv_depth, cv_depth_8U, 0, 256 * 256, NORM_MINMAX);
////cv_depth_8U.convertTo(cv_depth, CV_8U, 1);

//cv_irImage = cv::Mat(irImage.get_height_pixels(), irImage.get_width_pixels(), CV_16U,
//	(void*)irImage.get_buffer(), static_cast<size_t>(irImage.get_stride_bytes()));
//normalize(cv_irImage, cv_irImage_8U, 0, 256 * 256, NORM_MINMAX);
//cv_irImage.convertTo(cv_irImage_8U, CV_8U, 1);
//cv::imshow("depth", cv_depth_8U);

/*
static void depth_to_rgb_mode(k4a::device & device, size_t device_id, const k4a::capture & capture)
{
	k4a_device_configuration_t config;
	// 深度转RGB模式
	k4a::calibration k4aCalibration = device.get_calibration(config.depth_mode,
															 config.color_resolution);// Get the camera calibration for the entire K4A device, which is used for all transformation functions.

	k4a::transformation k4aTransformation = k4a::transformation(k4aCalibration);

	k4a::image rgbImage = capture.get_color_image();
	k4a::image depthImage = capture.get_depth_image();
	k4a::image transformed_depthImage = k4aTransformation.depth_image_to_color_camera(depthImage);
	cv::Mat cv_rgbImage_with_alpha(rgbImage.get_height_pixels(), rgbImage.get_width_pixels(), CV_8UC4,
								   (void *) rgbImage.get_buffer());
	cv::cvtColor(cv_rgbImage_with_alpha, cv_rgbImage_no_alpha, cv::COLOR_BGRA2BGR);
	cv::Mat cv_depth(transformed_depthImage.get_height_pixels(), transformed_depthImage.get_width_pixels(), CV_16U,
					   (void *) transformed_depthImage.get_buffer(), static_cast<size_t>(transformed_depthImage.get_stride_bytes()));


	cv::Mat dst;
	cv::convertScaleAbs(cv_depth, dst, 0.08);
	cv::applyColorMap(dst, cv_depth_8U, cv::COLORMAP_JET);
	//cv::imshow("depth", cv_depth_8U);

}
*/

static void save_cv_mat(const cv::Mat & mat, const std::filesystem::path & path)
{
	std::filesystem::create_directories(path.parent_path());

	bool save_result = imwrite(path.string(), mat);
	if (!save_result) {
		throw std::runtime_error("Save failed.");
	}
}

static std::filesystem::path working_dir = R"(D:\dk.test\)";

/**
 * 一个相机的工作线程
 */
static void camera_working_thread(DKCamera & camera)
{
	std::filesystem::path camera_working_dir = working_dir / ("camera" + std::to_string(camera.device_id));

	std::filesystem::path depth_path_base = camera_working_dir / "depth1";
	std::filesystem::path depth_path2_base = camera_working_dir / "depth2";

	int count = 0;
	while (true) {
		k4a::capture capture;
		// device.get_capture(&capture, std::chrono::milliseconds(0))

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

		cv::Mat cv_rgbImage_no_alpha = rgb(capture);
		cv::Mat cv_depth = depth(capture);

		std::string date = current_date();
		std::string timestamp = current_timestamp();

		std::filesystem::path filename_rgb = depth_path_base / date / (timestamp + ".png");
		try {
			save_cv_mat(cv_rgbImage_no_alpha, filename_rgb);
		} catch (...) {
			KERBAL_LOG_WRITE(KERROR, "Camera {}: rgb save failed: {}", camera.device_id, filename_rgb.string());
		}

		std::filesystem::path filename_depth = depth_path2_base / date / (timestamp + ".jpg");
		try {
			save_cv_mat(cv_depth, filename_depth);
		} catch (...) {
			KERBAL_LOG_WRITE(KERROR, "Camera {}: depth save failed: {}", camera.device_id, filename_depth.string());
		}

		//imwrite(folderPath2+ filename_d2, cv_depth_8U);

		count++;
		KERBAL_LOG_WRITE(KDEBUG, "Camera {}: Frame {} handle done.", camera.device_id, count);

		cv::waitKey(1);

	}

}

int main(int argc, char * argv[])
{
	// 找到并打开 Azure Kinect 设备
	uint32_t device_count = k4a::device::get_installed_count();    // 发现已连接的设备数
	if (0 == device_count) {
		KERBAL_LOG_WRITE(KFATAL, "No camera found.");
		exit(EXIT_FAILURE);
	}
	if (2 != device_count) {
		KERBAL_LOG_WRITE(KFATAL, "K4A camera count != 2.");
		exit(EXIT_FAILURE);
	}

	KERBAL_LOG_WRITE(KINFO, "Found {} connected cameras.", device_count);


	// 打开设备
	std::vector<DKCamera> cameras;
	cameras.reserve(device_count);

	try {
		cameras.emplace_back(k4a::device::open(0), 0, get_config_0());
	} catch (...) {
		KERBAL_LOG_WRITE(KFATAL, "Camara {} open failed.", 0);
		exit(EXIT_FAILURE);
	}

	try {
		cameras.emplace_back(k4a::device::open(1), 1, get_config_1());
	} catch (...) {
		KERBAL_LOG_WRITE(KFATAL, "Camara {} open failed.", 1);
		exit(EXIT_FAILURE);
	}

	// 启动设备
	for (size_t i = 0; i < cameras.size(); ++i) {
		try {
			cameras[i].start();
		} catch (...) {
			KERBAL_LOG_WRITE(KFATAL, "Camara {} start failed.", i);
			exit(EXIT_FAILURE);
		}
	}

	KERBAL_LOG_WRITE(KINFO, "{} cameras have started.", device_count);


	// 检索 Azure Kinect 图像数据
	for (size_t i = 0; i < cameras.size(); ++i) {
		cameras[i].stable();
	}


	std::vector<std::thread> threads(cameras.size());
	for (size_t i = 0; i < cameras.size(); ++i) {
		threads[i] = std::thread(camera_working_thread, std::ref(cameras[i]));
	}

	for (std::thread & thread : threads) {
		thread.join();
	}

	cv::destroyAllWindows();

	KERBAL_LOG_WRITE(KINFO, "Good Bye!");

	return EXIT_SUCCESS;
}
