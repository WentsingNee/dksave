// C++
#include <iostream>
// OpenCV
#include <opencv2/opencv.hpp>
// Kinect DK
#include <k4a/k4a.hpp>
#include<windows.h>
#include <time.h>
#include <fstream>
#include <vector>
#include <iomanip>
#include <sstream>
#include <shlwapi.h>
#include <ratio>   //表示时间单位的库
#include <chrono>
#include<ctime>
// #include <pcl/console/time.h>   // 利用控制台计算时间

// 宏
// 方便控制是否 std::cout 信息
#define DEBUG_std_cout 0

using namespace cv;
using namespace std;
static int flag = 1;
static int a = 0;

int main(int argc, char* argv[])
{
	/*
		找到并打开 Azure Kinect 设备
	*/
	// 发现已连接的设备数
	const uint32_t device_count = k4a::device::get_installed_count();
	if (0 == device_count)
	{
		std::cout << "Error: no K4A devices found. " << std::endl;
		return EXIT_FAILURE;
	}
	else
	{
		std::cout << "Found " << device_count << " connected devices. " << std::endl;

		/*if (1 != device_count)// 超过1个设备，也输出错误信息。
		{
			std::cout << "Error: more than one K4A devices found. " << std::endl;
			return EXIT_FAILURE;
		}
		else// 该示例代码仅限对1个设备操作
		{
			std::cout << "Done: found 1 K4A device. " << std::endl;
		}*/
	}
	// 打开（默认）设备
	k4a::device device = k4a::device::open(1);
	std::cout << "Done: open device. " << std::endl;


	/*
		检索 Azure Kinect 图像数据
	*/
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
	std::cout << "Done: start camera." << std::endl;

	// 稳定化
	k4a::capture capture;
	int iAuto = 0;//用来稳定，类似自动曝光
	int iAutoError = 0;// 统计自动曝光的失败次数

	string defaultPath = "D:\\database_center\\depth1\\";
	string defaultPath2 = "D:\\database_center\\彩色深度图像1\\";

	char pszKnownPath[30];
	char pa[30];
	SYSTEMTIME lpsystime;
	GetLocalTime(&lpsystime);
	sprintf_s(pszKnownPath, "%04d-%02d-%02d\\", lpsystime.wYear, lpsystime.wMonth, lpsystime.wDay);


	if (flag)
	{
		fstream _file;
		_file.open(".\\text.txt", ios::in);
		if (!_file)
		{
			ofstream fout1(".\\text.txt");//建立名为text.txt的文件并与fout关联;
			fout1 << a << endl;
			fout1.close();

		}

		ifstream fin(".\\text.txt", ios::in);
		int str1;
		fin >> str1;
		a = str1;
		fin.close();

		a = a + 1;
		cout << a << endl;
		sprintf_s(pa, "%04d\\", a);
		string command;
		string folderPath = defaultPath + pszKnownPath + pa;
		string folderPath2 = defaultPath2 + pszKnownPath + pa;
		FILE* pFile;
		//pFile = fopen(folderPath, "r");
		//if (pFile == NULL)
		//{

		command = "mkdir -p " + folderPath;
		system(command.c_str());
		command = "mkdir -p " + folderPath2;
		system(command.c_str());
		//}
		ofstream fout(".\\text.txt");//建立名为text.txt的文件并与fout关联;
		fout << a << endl;
		fout.close();

		flag = 0;
	}

	string folderPath = defaultPath + pszKnownPath + pa;
	string folderPath2 = defaultPath2 + pszKnownPath + pa;
	while (true)
	{
		if (device.get_capture(&capture))
		{
			std::cout << iAuto << ". Capture several frames to give auto-exposure" << std::endl;

			// 跳过前 n 个（成功的数据采集）循环，用来稳定
			if (iAuto != 30)
			{
				iAuto++;
				continue;
			}
			else
			{
				std::cout << "Done: auto-exposure" << std::endl;
				break;// 跳出该循环，完成相机的稳定过程
			}
		}
		else
		{
			std::cout << iAutoError << ". K4A_WAIT_RESULT_TIMEOUT." << std::endl;
			if (iAutoError != 30)
			{
				iAutoError++;
				continue;
			}
			else
			{
				std::cout << "Error: failed to give auto-exposure. " << std::endl;
				return EXIT_FAILURE;
			}
		}
	}
	std::cout << "-----------------------------------" << std::endl;
	std::cout << "----- Have Started Kinect DK. -----" << std::endl;
	std::cout << "-----------------------------------" << std::endl;


	// 从设备获取捕获
	k4a::image rgbImage;
	k4a::image depthImage;
	k4a::image irImage;
	k4a::image transformed_depthImage;
	k4a::image transformed_colorImage;

	cv::Mat cv_rgbImage_with_alpha;
	cv::Mat cv_rgbImage_no_alpha;
	cv::Mat cv_depth;
	cv::Mat cv_depth_8U;
	cv::Mat cv_irImage;
	cv::Mat cv_irImage_8U;
	//pcl::console::TicToc time;
	//time.tic();
	int length = 50;
	vector<Mat> vrgb;
	vector<Mat> vdepth;
	vector<string> vpath;
	int count = 0;
	while (true)
		// for (size_t i = 0; i < 100; i++)
	{
		// if (device.get_capture(&capture, std::chrono::milliseconds(0)))
		if (device.get_capture(&capture))
		{			
			// rgb
			// * Each pixel of BGRA32 data is four bytes. The first three bytes represent Blue, Green,
			// * and Red data. The fourth byte is the alpha channel and is unused in the Azure Kinect APIs.
			auto t0 = chrono::system_clock::now();
			rgbImage = capture.get_color_image();

#if DEBUG_std_cout == 1
			std::cout << "[rgb] " << "\n"
				<< "format: " << rgbImage.get_format() << "\n"
				<< "device_timestamp: " << rgbImage.get_device_timestamp().count() << "\n"
				<< "system_timestamp: " << rgbImage.get_system_timestamp().count() << "\n"
				<< "height*width: " << rgbImage.get_height_pixels() << ", " << rgbImage.get_width_pixels()
				<< std::endl;
#endif
			cv_rgbImage_with_alpha = cv::Mat(rgbImage.get_height_pixels(), rgbImage.get_width_pixels(), CV_8UC4, (void*)rgbImage.get_buffer());
			cv::cvtColor(cv_rgbImage_with_alpha, cv_rgbImage_no_alpha, cv::COLOR_BGRA2BGR);

			// depth
			// * Each pixel of DEPTH16 data is two bytes of little endian unsigned depth data. The unit of the data is in
			// * millimeters from the origin of the camera.
			depthImage = capture.get_depth_image();
#if DEBUG_std_cout == 1
			std::cout << "[depth] " << "\n"
				<< "format: " << depthImage.get_format() << "\n"
				<< "device_timestamp: " << depthImage.get_device_timestamp().count() << "\n"
				<< "system_timestamp: " << depthImage.get_system_timestamp().count() << "\n"
				<< "height*width: " << depthImage.get_height_pixels() << ", " << depthImage.get_width_pixels()
				<< std::endl;
#endif
			cv_depth = cv::Mat(depthImage.get_height_pixels(), depthImage.get_width_pixels(), CV_16U, depthImage.get_buffer());
			normalize(cv_depth, cv_depth_8U, 0, 256 * 256, NORM_MINMAX);
			cv_depth.convertTo(cv_depth_8U, CV_8U);

			// ir
			// * Each pixel of IR16 data is two bytes of little endian unsigned depth data. The value of the data represents
			// * brightness.
			//irImage = capture.get_ir_image();
#if DEBUG_std_cout == 1
			std::cout << "[ir] " << "\n"
				<< "format: " << irImage.get_format() << "\n"
				<< "device_timestamp: " << irImage.get_device_timestamp().count() << "\n"
				<< "system_timestamp: " << irImage.get_system_timestamp().count() << "\n"
				<< "height*width: " << irImage.get_height_pixels() << ", " << irImage.get_width_pixels()
				<< std::endl;
#endif
			//cv_irImage = cv::Mat(irImage.get_height_pixels(), irImage.get_width_pixels(), CV_16U, (void*)irImage.get_buffer());
			//cv_irImage.convertTo(cv_irImage_8U, CV_8U, 1);

			//深度图和RGB图配准
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
			
			// 深度转RGB模式
			k4a::calibration k4aCalibration = device.get_calibration(config.depth_mode, config.color_resolution);// Get the camera calibration for the entire K4A device, which is used for all transformation functions.
			
			
			
			k4a::transformation k4aTransformation = k4a::transformation(k4aCalibration);
			

			transformed_depthImage = k4aTransformation.depth_image_to_color_camera(depthImage);
			cv_rgbImage_with_alpha = cv::Mat(rgbImage.get_height_pixels(), rgbImage.get_width_pixels(), CV_8UC4,
				(void*)rgbImage.get_buffer());
			cv::cvtColor(cv_rgbImage_with_alpha, cv_rgbImage_no_alpha, cv::COLOR_BGRA2BGR);
			cv_depth = cv::Mat(transformed_depthImage.get_height_pixels(), transformed_depthImage.get_width_pixels(), CV_16U,
				(void*)transformed_depthImage.get_buffer(), static_cast<size_t>(transformed_depthImage.get_stride_bytes()));

			

			Mat dst;
			cv::convertScaleAbs(cv_depth, dst, 0.08);
			cv::applyColorMap(dst, cv_depth_8U, COLORMAP_JET);
			//cv::imshow("depth", cv_depth_8U);
			
		/*	auto tt = chrono::system_clock::now();
			auto dt = chrono::duration_cast<chrono::duration<float>>(tt - t0).count();

			cout << dt << endl;*/

			SYSTEMTIME lpsystime1;
			GetLocalTime(&lpsystime1);
			char szTimeString1[30];
			sprintf_s(szTimeString1, "%04d-%02d-%02d_%02d-%02d-%02d-%03d", lpsystime1.wYear, lpsystime1.wMonth,
				lpsystime1.wDay, lpsystime1.wHour, lpsystime1.wMinute, lpsystime1.wSecond, lpsystime1.wMilliseconds);
			string szTimeStrin1 = szTimeString1;
			std::string filename_d = szTimeStrin1 + ".png";
			std::string filename_d2 = szTimeStrin1 + ".jpg";
			std::cout << filename_d2 << std::endl;
			vrgb.push_back(cv_rgbImage_no_alpha.clone());
			vdepth.push_back(cv_depth.clone());
			vpath.push_back(filename_d);
			count++;
			/*if (count == 50) {
				for (int i = 0; i < vpath.size(); i++)
				{
					imwrite(folderPath + vpath[i], vdepth[i]);

					imwrite(folderPath2 + vpath[i], vrgb[i]);
				}
				vdepth.clear();
				vrgb.clear();
				vpath.clear();
				count = 0;
				
			}*/
			imwrite(folderPath + filename_d, cv_depth);

			imwrite(folderPath2 + filename_d, cv_rgbImage_no_alpha);
			//imwrite(folderPath2+ filename_d2, cv_depth_8U); 
			
			cv::waitKey(1);

			std::cout << "--- test ---" << std::endl;

		}

	}

	cv::destroyAllWindows();

	// 释放，关闭设备
	rgbImage.reset();
	depthImage.reset();
	irImage.reset();
	capture.reset();
	device.close();



	// ---------------------------------------------------------------------------------------------------------
	/*
				Test
	*/
	// 等待输入，方便显示上述运行结果
	std::cout << "--------------------------------------------" << std::endl;
	std::cout << "Waiting for inputting an integer: ";
	int wd_wait;
	std::cin >> wd_wait;

	std::cout << "----------------------------------" << std::endl;
	std::cout << "------------- closed -------------" << std::endl;
	std::cout << "----------------------------------" << std::endl;

	return EXIT_SUCCESS;
}