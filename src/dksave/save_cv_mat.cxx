/**
 * @file       save_cv_mat.cxx
 * @brief
 * @date       2023-09-24
 * @author     Wentsing Nee
 * @copyright
 *      Wentsing Nee of China Agricultural University
 *   all rights reserved
 */

module;

#include "dksave/logger.hpp"

#include <filesystem>
#include <stdexcept>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

export module dksave.save_cv_mat;

import dksave.logger;


namespace dksave
{

	/**
	 * 将一个 cv::Mat 矩阵保存在指定路径. 若指定路径的上层文件夹不存在, 则会自动创建
	 */
	export
	inline
	void save_cv_mat(cv::Mat const & mat, std::filesystem::path const & path, std::vector<int> const & params = {})
	{
		std::filesystem::path dir(path.parent_path());
		try {
			std::filesystem::create_directories(dir);
		} catch (std::exception const & e) {
			KERBAL_LOG_WRITE(
				KERROR, "Create directory failed. directory: {}, exception_type: {}, what: {}",
				dir.string(),
				typeid(e).name(), e.what()
			);
			throw;
		}

		bool write_success = cv::imwrite(path.string(), mat, params);
		if (!write_success) {
			KERBAL_LOG_WRITE(KERROR, "Image saved failed. path: {}", dir.string());
			throw std::runtime_error("Image saved failed");
		}

		KERBAL_LOG_WRITE(KVERBOSE, "Image saved. path: {}", path.string());
	}

} // namespace dksave
