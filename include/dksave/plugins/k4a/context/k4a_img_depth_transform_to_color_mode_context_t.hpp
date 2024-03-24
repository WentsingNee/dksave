/**
 * @file       k4a_img_depth_transform_to_color_mode_context_t.hpp
 * @brief
 * @date       2023-06-30
 * @author     Wentsing Nee
 * @copyright
 *      Wentsing Nee of China Agricultural University
 *   all rights reserved
 */

#ifndef DKSAVE_PLUGINS_K4A_CONTEXT_K4A_IMG_DEPTH_TRANSFORM_TO_COLOR_MODE_CONTEXT_T_HPP
#define DKSAVE_PLUGINS_K4A_CONTEXT_K4A_IMG_DEPTH_TRANSFORM_TO_COLOR_MODE_CONTEXT_T_HPP

#include <k4a/k4a.hpp>


/**
 * 本类负责将一张深度图片向可见光模式配准
 */
struct k4a_img_depth_transform_to_color_mode_context_t
{
		k4a::image k4a_img_depth_transformed_to_color;

		k4a::image & transform(const k4a::transformation & transformation, const k4a::image & k4a_img_depth)
		{
			this->k4a_img_depth_transformed_to_color = transformation.depth_image_to_color_camera(k4a_img_depth);
			return this->k4a_img_depth_transformed_to_color;
		}
};

#endif // DKSAVE_PLUGINS_K4A_CONTEXT_K4A_IMG_DEPTH_TRANSFORM_TO_COLOR_MODE_CONTEXT_T_HPP
