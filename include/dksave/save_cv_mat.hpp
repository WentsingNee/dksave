/**
 * @file       save_cv_mat.hpp
 * @brief
 * @date       2023-09-24
 * @author     Wentsing Nee
 * @copyright
 *      Wentsing Nee of China Agricultural University
 *   all rights reserved
 */

#ifndef DKSAVE_SAVE_CV_MAT_HPP
#define DKSAVE_SAVE_CV_MAT_HPP

#include "logger.hpp"

#include <filesystem>
#include <stdexcept>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>


/**
 * 将一个 cv::Mat 矩阵保存在指定路径. 若指定路径的上层文件夹不存在, 则会自动创建
 */
inline
void save_cv_mat(cv::Mat const & mat, std::filesystem::path const & path, std::vector<int> const & params = {})
{
	std::filesystem::create_directories(path.parent_path());

	bool write_success = cv::imwrite(path.string(), mat, params);
	if (!write_success) {
		throw std::runtime_error("Save failed.");
	}

	KERBAL_LOG_WRITE(KVERBOSE, "Saved {}", path.string());
}

#endif // DKSAVE_SAVE_CV_MAT_HPP
