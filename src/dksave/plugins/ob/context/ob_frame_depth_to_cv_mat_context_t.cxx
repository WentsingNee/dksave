/**
 * @file       ob_frame_depth_to_cv_mat_context_t.cxx
 * @brief
 * @date       2024-09-12
 * @author     Peter
 * @copyright
 *      Peter of [ThinkSpirit Laboratory](http://thinkspirit.org/)
 *   of [Nanjing University of Information Science & Technology](http://www.nuist.edu.cn/)
 *   all rights reserved
 */

module;

#include <libobsensor/hpp/Frame.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>


export module dksave.plugins.ob.context.ob_frame_depth_to_cv_mat_context_t;


namespace dksave::plugins_ob
{

	export
	struct ob_frame_depth_to_cv_mat_context_t
	{
			cv::Mat cv_mat;

			cv::Mat const &
			transform(
				std::shared_ptr<ob::Frame> in_color,
				uint32_t color_height, uint32_t color_width
			)
			{
				this->cv_mat = cv::Mat(color_height, color_width, CV_16U, in_color->data());
				return this->cv_mat;
			}

	};

} // namespace dksave::plugins_ob
