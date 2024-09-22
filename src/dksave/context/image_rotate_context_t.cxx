/**
 * @file       image_rotate_context_t.cxx
 * @brief
 * @date       2024-09-12
 * @author     Wentsing Nee
 * @copyright
 *      Wentsing Nee of China Agricultural University
 *   all rights reserved
 */

module;

#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc/imgproc.hpp>

export module dksave.context.image_rotate_context_t;

import dksave.rotate_flag_t;


namespace dksave
{

	export
	struct image_rotate_context_t
	{
			cv::Mat rotated;

			cv::Mat const &
			rotate(cv::Mat const & input, dksave::rotate_flag_t x)
			{
				switch (x) {
					case dksave::rotate_flag_t::CLOCKWISE_0: {
						this->rotated = input;
						break;
					}
					case dksave::rotate_flag_t::CLOCKWISE_90: {
						cv::rotate(input, this->rotated, cv::ROTATE_90_CLOCKWISE);
						break;
					}
					case dksave::rotate_flag_t::CLOCKWISE_180: {
						cv::rotate(input, this->rotated, cv::ROTATE_180);
						break;
					}
					case dksave::rotate_flag_t::CLOCKWISE_270: {
						cv::rotate(input, this->rotated, cv::ROTATE_90_COUNTERCLOCKWISE);
						break;
					}
				}
				return this->rotated;
			}

	};

} // namespace dksave
