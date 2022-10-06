/**
 * @file       DKSave.cpp
 * @brief
 * @date       2022-10-06
 * @author     Peter
 */

#include <iostream>
#include <fstream>
#include <vector>
#include <iomanip>
#include <sstream>
#include <ratio>
#include <chrono>
#include <ctime>
#include <filesystem>

// OpenCV
#include <opencv2/opencv.hpp>

// Kinect DK
#include <k4a/k4a.hpp>
#include <windows.h>
#include <shlwapi.h>

#include "logger.hpp"


static void configure_and_start_camara1(k4a::device & device, size_t device_id)
{
	// 配置并启动设备
	k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
	config.camera_fps = K4A_FRAMES_PER_SECOND_30;
	config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;

	config.color_resolution = K4A_COLOR_RESOLUTION_720P; //1536P
	//config.color_resolution = K4A_COLOR_RESOLUTION_1536P;

	//config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
	//config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
	config.depth_mode = K4A_DEPTH_MODE_WFOV_2X2BINNED;
	//config.depth_mode = K4A_DEPTH_MODE_WFOV_UNBINNED;

	config.synchronized_images_only = true;// ensures that depth and color images are both available in the capture

	KERBAL_LOG_WRITE(INFO, "Done: start camera.");
}


static void configure_and_start_camara(k4a::device & device, size_t device_id)
{
	// 配置并启动设备
	k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
	config.camera_fps = K4A_FRAMES_PER_SECOND_15;
	config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;

	//config.color_resolution = K4A_COLOR_RESOLUTION_720P; //1536P
	config.color_resolution = K4A_COLOR_RESOLUTION_1536P;

	//config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
	//config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
	//config.depth_mode = K4A_DEPTH_MODE_WFOV_2X2BINNED;
	config.depth_mode = K4A_DEPTH_MODE_WFOV_UNBINNED;

	config.synchronized_images_only = true;// ensures that depth and color images are both available in the capture

	device.start_cameras(&config);
	KERBAL_LOG_WRITE(INFO, "Done: start camera.");
}

static void stable(k4a::device & device, size_t device_id)
{
	// 稳定化
	k4a::capture capture;
	int success = 0;//用来稳定，类似自动曝光
	int failed = 0;// 统计自动曝光的失败次数

	while (true) {
		if (device.get_capture(&capture)) {
			// 跳过前 n 个（成功的数据采集）循环，用来稳定
			success++;
			KERBAL_LOG_WRITE(INFO, "Capture several frames to give auto-exposure for {} times.", success);

			if (success == 30) {
				KERBAL_LOG_WRITE(INFO, "Done: auto-exposure.");
				break;// 跳出该循环，完成相机的稳定过程
			}
		} else {
			failed++;
			KERBAL_LOG_WRITE(WARNING, "K4A_WAIT_RESULT_TIMEOUT for {} times.", failed);

			if (failed == 30) {
				KERBAL_LOG_WRITE(FATAL, "Failed to give auto-exposure.", failed);
				exit(EXIT_FAILURE);
			}
		}
	}
}

static void prepare_working_dir()
{

}

static void destribe_image(const k4a::image & img, const char type[])
{
	KERBAL_LOG_WRITE(DEBUG, "[{}]", type);
	KERBAL_LOG_WRITE(DEBUG, "format: {}\n", rgbImage.get_format());
	KERBAL_LOG_WRITE(DEBUG, "device_timestamp: {}\n", rgbImage.get_device_timestamp().count());
	KERBAL_LOG_WRITE(DEBUG, "system_timestamp: {}\n", rgbImage.get_system_timestamp().count());
	KERBAL_LOG_WRITE(DEBUG, "height: {}, width: {}\n", rgbImage.get_height_pixels(), rgbImage.get_width_pixels());
}

static cv::Mat rgb(k4a::capture & capture)
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

static cv::Mat depth(k4a::capture & capture)
{
	// depth
	// * Each pixel of DEPTH16 data is two bytes of little endian unsigned depth data. The unit of the data is in
	// * millimeters from the origin of the camera.

	k4a::image depthImage = capture.get_depth_image();
	destribe_image(depthImage, "depth");

	cv::Mat cv_depth(depthImage.get_height_pixels(), depthImage.get_width_pixels(), CV_16U, depthImage.get_buffer());
	cv::Mat cv_depth_8U;
	normalize(cv_depth, cv_depth_8U, 0, 256 * 256, NORM_MINMAX);
	cv_depth.convertTo(cv_depth_8U, CV_8U);

	return cv_depth;
}

static void ir(k4a::capture & capture)
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


static void depth_to_rgb_mode(k4a::device & device, size_t device_id)
{
	// 深度转RGB模式
	k4a::calibration k4aCalibration = device.get_calibration(config.depth_mode,
															 config.color_resolution);// Get the camera calibration for the entire K4A device, which is used for all transformation functions.

	k4a::transformation k4aTransformation = k4a::transformation(k4aCalibration);
	k4a::image depthImage = capture.get_depth_image();
	k4a::image transformed_depthImage = k4aTransformation.depth_image_to_color_camera(depthImage);
	cv::Mat cv_rgbImage_with_alpha(rgbImage.get_height_pixels(), rgbImage.get_width_pixels(), CV_8UC4,
								   (void *) rgbImage.get_buffer());
	cv::cvtColor(cv_rgbImage_with_alpha, cv_rgbImage_no_alpha, cv::COLOR_BGRA2BGR);
	cv::Mat cv_depth(transformed_depthImage.get_height_pixels(), transformed_depthImage.get_width_pixels(), CV_16U,
					   (void *) transformed_depthImage.get_buffer(), static_cast<size_t>(transformed_depthImage.get_stride_bytes()));


	cv::Mat dst;
	cv::convertScaleAbs(cv_depth, dst, 0.08);
	cv::applyColorMap(dst, cv_depth_8U, COLORMAP_JET);
	//cv::imshow("depth", cv_depth_8U);

}

int main(int argc, char * argv[])
{
	/*
		找到并打开 Azure Kinect 设备
	*/
	// 发现已连接的设备数
	uint32_t device_count = k4a::device::get_installed_count();
	if (0 == device_count) {
		KERBAL_LOG_WRITE(FATAL, "No K4A device found.");
		exit(EXIT_FAILURE);
	}

	KERBAL_LOG_WRITE(INFO, "Found {} connected devices.", device_count);

	// 打开设备
	std::vector<k4a::device> devices;
	devices.reserve(device_count);
	for (uint32_t i = 0; i < device_count; ++i) {
		devices.push_back(k4a::device::open(1));
		configure_and_start_camara(devices[i], i);
	}
	KERBAL_LOG_WRITE(INFO, "Done: open device.", device_count);

	/*
		检索 Azure Kinect 图像数据
	*/

	for (size_t i = 0; i < devices.size(); ++i) {
		stable(devices[i], i);
	}


	const char working_dir[] = R"(D:\database_center\)";

	string defaultPath = "D:\\database_center\\depth1\\";
	string defaultPath2 = "D:\\database_center\\彩色深度图像1\\";

	char pszKnownPath[30];
	char pa[30];
	SYSTEMTIME lpsystime;
	GetLocalTime(&lpsystime);
	sprintf_s(pszKnownPath, "%04d-%02d-%02d\\", lpsystime.wYear, lpsystime.wMonth, lpsystime.wDay);

	string folderPath = defaultPath + pszKnownPath + pa;
	string folderPath2 = defaultPath2 + pszKnownPath + pa;
	std::cout << "-----------------------------------" << std::endl;
	std::cout << "----- Have Started Kinect DK. -----" << std::endl;
	std::cout << "-----------------------------------" << std::endl;


	int count = 0;
	while (true) {
		k4a::capture capture;
		// if (device.get_capture(&capture, std::chrono::milliseconds(0)))
		if (device.get_capture(&capture)) {
			auto t0 = chrono::system_clock::now();


			string szTimeStrin1 = current_timestamp();
			std::string filename_d = szTimeStrin1 + ".png";
			std::string filename_d2 = szTimeStrin1 + ".jpg";
			std::cout << filename_d2 << std::endl;

			imwrite(folderPath + filename_d, cv_depth);

			imwrite(folderPath2 + filename_d, cv_rgbImage_no_alpha);
			//imwrite(folderPath2+ filename_d2, cv_depth_8U);

			count++;
			KERBAL_LOG_WRITE(DEBUG, "Once.");

			cv::waitKey(1);

		}

	}

	cv::destroyAllWindows();

	for (size_t i = 0; i < devices.size(); ++i) {
		devices[i].close();
	}

	KERBAL_LOG_WRITE(INFO, "Good Bye!");

	return EXIT_SUCCESS;
}