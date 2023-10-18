/**
 * @file       k4a_img_color_to_cv_mat_context_t.hpp
 * @brief
 * @date       2022-10-12
 * @author     Wentsing Nee
 * @copyright
 *      Wentsing Nee of China Agricultural University
 *   all rights reserved
 */

#ifndef DKSAVE_K4A_IMG_COLOR_TO_CV_MAT_CONTEXT_T_HPP
#define DKSAVE_K4A_IMG_COLOR_TO_CV_MAT_CONTEXT_T_HPP

#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc.hpp>
#include <k4a/k4a.hpp>


/**
 * 本类负责将 k4a:image 格式的可见光图片转换为不带透明度通道的 cv::Mat 格式
 */
struct k4a_img_color_to_cv_mat_context_t
{
		cv::Mat cv_color_img_with_alpha;
		cv::Mat cv_color_img_without_alpha;

		/**
		 * @param k4a_img_color 一般为通过 capture.get_color_image() 得到的图像
		 */
		cv::Mat & convert(const k4a::image & k4a_img_color)
		{
			// color
			// * Each pixel of BGRA32 data is four bytes. The first three bytes represent Blue, Green,
			// * and Red data. The fourth byte is the alpha channel and is unused in the Azure Kinect APIs.

			this->cv_color_img_with_alpha = cv::Mat(k4a_img_color.get_height_pixels(), k4a_img_color.get_width_pixels(), CV_8UC4, (void *)(k4a_img_color.get_buffer()));
			cv::cvtColor(this->cv_color_img_with_alpha, this->cv_color_img_without_alpha, cv::COLOR_BGRA2BGR);
			return this->cv_color_img_without_alpha;
		}
};

#endif // DKSAVE_K4A_IMG_COLOR_TO_CV_MAT_CONTEXT_T_HPP
