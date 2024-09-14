/**
 * @file       image_rotate_context_t.hpp
 * @brief
 * @date       2024-09-12
 * @author     Wentsing Nee
 * @copyright
 *      Wentsing Nee of China Agricultural University
 *   all rights reserved
 */

#ifndef DKSAVE_CONTEXT_IMAGE_ROTATE_CONTEXT_T_HPP
#define DKSAVE_CONTEXT_IMAGE_ROTATE_CONTEXT_T_HPP

#include "dksave/rotate_flag_t.hpp"

#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc/imgproc.hpp>


namespace dksave
{

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

#endif // DKSAVE_CONTEXT_IMAGE_ROTATE_CONTEXT_T_HPP
