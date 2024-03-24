/**
 * @file       k4a_img_depth_to_cv_mat_context_t.hpp
 * @brief
 * @date       2023-06-30
 * @author     Wentsing Nee
 * @copyright
 *      Wentsing Nee of China Agricultural University
 *   all rights reserved
 */

#ifndef DKSAVE_PLUGINS_K4A_CONTEXT_K4A_IMG_DEPTH_TO_CV_MAT_CONTEXT_T_HPP
#define DKSAVE_PLUGINS_K4A_CONTEXT_K4A_IMG_DEPTH_TO_CV_MAT_CONTEXT_T_HPP

#include <opencv2/core/mat.hpp>
#include <k4a/k4a.hpp>


/**
 * 本类负责将 k4a:image 格式的深度图片转换为 cv::Mat 格式
 */
struct k4a_img_depth_to_cv_mat_context_t
{
		cv::Mat cv_depth_img;

		cv::Mat & convert(const k4a::image & k4a_img_depth)
		{
			this->cv_depth_img = cv::Mat(k4a_img_depth.get_height_pixels(), k4a_img_depth.get_width_pixels(), CV_16U,
				(void*)(k4a_img_depth.get_buffer()), static_cast<size_t>(k4a_img_depth.get_stride_bytes()));

			return this->cv_depth_img;
		}
};

#endif // DKSAVE_PLUGINS_K4A_CONTEXT_K4A_IMG_DEPTH_TO_CV_MAT_CONTEXT_T_HPP
