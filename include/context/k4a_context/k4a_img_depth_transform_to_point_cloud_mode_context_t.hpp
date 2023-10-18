/**
 * @file       k4a_img_depth_transform_to_point_cloud_mode_context_t.hpp
 * @brief
 * @date       2023-06-30
 * @author     Wentsing Nee
 * @copyright
 *      Wentsing Nee of China Agricultural University
 *   all rights reserved
 */

#ifndef DKSAVE_K4A_IMG_DEPTH_TRANSFORM_TO_POINT_CLOUD_MODE_CONTEXT_T_HPP
#define DKSAVE_K4A_IMG_DEPTH_TRANSFORM_TO_POINT_CLOUD_MODE_CONTEXT_T_HPP

#include <k4a/k4a.hpp>


/**
 * 本类负责将一张深度图片转换为点云格式
 */
struct k4a_img_depth_transform_to_point_cloud_mode_context_t
{
		k4a::image k4a_img_point_cloud;

		/**
		 * @param k4a_img_depth 必须是是已经配准为可见光模式的深度图像
		 */
		k4a::image & transform(const k4a::transformation & transformation, const k4a::image & k4a_img_depth)
		{
			this->k4a_img_point_cloud = transformation.depth_image_to_point_cloud(
					k4a_img_depth, K4A_CALIBRATION_TYPE_COLOR);
			return this->k4a_img_point_cloud;
		}
};

#endif // DKSAVE_K4A_IMG_DEPTH_TRANSFORM_TO_POINT_CLOUD_MODE_CONTEXT_T_HPP
