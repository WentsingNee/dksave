/**
 * @file       ob_color_frame_to_cv_mat_context_t.hpp
 * @brief
 * @date       2024-05-18
 * @author     Peter
 * @copyright
 *      Peter of [ThinkSpirit Laboratory](http://thinkspirit.org/)
 *   of [Nanjing University of Information Science & Technology](http://www.nuist.edu.cn/)
 *   all rights reserved
 */

#ifndef DKSAVE_PLUGINS_OB_CONTEXT_OB_COLOR_FRAME_TO_CV_MAT_CONTEXT_T_HPP
#define DKSAVE_PLUGINS_OB_CONTEXT_OB_COLOR_FRAME_TO_CV_MAT_CONTEXT_T_HPP

#include <libobsensor/hpp/Frame.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>


namespace dksave::plugins_ob
{

	struct ob_color_frame_to_cv_mat_context_t
	{
			cv::Mat cv_mat;

			cv::Mat const & cast(std::shared_ptr<ob::ColorFrame> color_frame)
			{
				this->cv_mat = cv::Mat(
					color_frame->height(),
					color_frame->width(),
					CV_8UC3,
					color_frame->data()
				);
				cv::cvtColor(this->cv_mat, this->cv_mat, cv::COLOR_RGB2BGR);
				return this->cv_mat;
			}

	};

} // namespace dksave::plugins_ob

#endif // DKSAVE_PLUGINS_OB_CONTEXT_OB_COLOR_FRAME_TO_CV_MAT_CONTEXT_T_HPP
