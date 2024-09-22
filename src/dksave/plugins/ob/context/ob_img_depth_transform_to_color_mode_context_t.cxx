/**
 * @file       ob_img_depth_transform_to_color_mode_context_t.cxx
 * @brief
 * @date       2024-05-18
 * @author     Peter
 * @copyright
 *      Peter of [ThinkSpirit Laboratory](http://thinkspirit.org/)
 *   of [Nanjing University of Information Science & Technology](http://www.nuist.edu.cn/)
 *   all rights reserved
 */

module;

#include <libobsensor/hpp/Device.hpp>
#include <libobsensor/hpp/Frame.hpp>
#include <libobsensor/hpp/Utils.hpp>


export module dksave.plugins.ob.context.ob_img_depth_transform_to_color_mode_context_t;


namespace dksave::plugins_ob
{

	export
	struct ob_img_depth_transform_to_color_mode_context_t
	{
			std::shared_ptr<ob::Frame> in_color;

			std::shared_ptr<ob::Frame> const &
			transform(
				std::shared_ptr<ob::Device> device,
				std::shared_ptr<ob::DepthFrame> depth_frame,
				uint32_t color_height, uint32_t color_width
			)
			{
				this->in_color = ob::CoordinateTransformHelper::transformationDepthFrameToColorCamera(
					device, depth_frame, color_width, color_height
				);
				return this->in_color;
			}

	};

} // namespace dksave::plugins_ob
