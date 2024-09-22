/**
 * @file       k4a_img_ir_to_cv_mat_context_t.hpp
 * @brief
 * @date       2023-06-30
 * @author     Wentsing Nee
 * @copyright
 *      Wentsing Nee of China Agricultural University
 *   all rights reserved
 */

#ifndef DKSAVE_PLUGINS_K4A_CONTEXT_K4A_IMG_IR_TO_CV_MAT_CONTEXT_T_HPP
#define DKSAVE_PLUGINS_K4A_CONTEXT_K4A_IMG_IR_TO_CV_MAT_CONTEXT_T_HPP

#include <opencv2/core/mat.hpp>
#include <k4a/k4a.hpp>


namespace dksave::plugins_k4a
{

	/**
	 * 本类负责将 k4a:image 格式的红外图片转换为 cv::Mat 格式
	 */
	struct k4a_img_ir_to_cv_mat_context_t
	{
			cv::Mat cv_ir_16u;
			cv::Mat cv_ir_8u;

			cv::Mat & convert(const k4a::image & k4a_img_ir)
			{
				// ir
				// * Each pixel of IR16 data is two bytes of little endian unsigned depth data. The value of the data represents
				// * brightness.

				this->cv_ir_16u = cv::Mat(
					k4a_img_ir.get_height_pixels(),
					k4a_img_ir.get_width_pixels(),
					CV_16U,
					const_cast<void *>(
						reinterpret_cast<void const *>(
							k4a_img_ir.get_buffer()
						)
					)
				);
				this->cv_ir_16u.convertTo(this->cv_ir_8u, CV_8U);
				return this->cv_ir_8u;
			}
	};

} // namespace dksave::plugins_k4a

#endif // DKSAVE_PLUGINS_K4A_CONTEXT_K4A_IMG_IR_TO_CV_MAT_CONTEXT_T_HPP
