/**
 * @file       k4a_img_color_transform_to_depth_mode_context_t.cxx
 * @brief
 * @date       2024-09-12
 * @author     Wentsing Nee
 * @copyright
 *      Wentsing Nee of China Agricultural University
 *   all rights reserved
 */

module;

#include <k4a/k4a.hpp>

export module dksave.plugins.k4a.context.k4a_img_color_transform_to_depth_mode_context_t;


namespace dksave::plugins_k4a
{

	/**
	 * 本类负责将一张可见光图片向深度模式配准
	 */
	export
	struct k4a_img_color_transform_to_depth_mode_context_t
	{
			k4a::image k4a_img_color_transformed_to_depth;

			k4a::image & transform(
				const k4a::transformation & transformation,
				const k4a::image & k4a_img_depth,
				const k4a::image & k4a_img_color
			)
			{
				this->k4a_img_color_transformed_to_depth = transformation.color_image_to_depth_camera(
					k4a_img_depth,
					k4a_img_color
				);
				return this->k4a_img_color_transformed_to_depth;
			}
	};

} // namespace dksave::plugins_k4a
