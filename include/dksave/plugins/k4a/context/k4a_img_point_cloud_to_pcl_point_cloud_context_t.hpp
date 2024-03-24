/**
 * @file       k4a_img_point_cloud_to_pcl_point_cloud_context_t.hpp
 * @brief
 * @date       2023-06-30
 * @author     Wentsing Nee
 * @copyright
 *      Wentsing Nee of China Agricultural University
 *   all rights reserved
 */

#ifndef DKSAVE_PLUGINS_K4A_CONTEXT_K4A_IMG_POINT_CLOUD_TO_PCL_POINT_CLOUD_CONTEXT_T_HPP
#define DKSAVE_PLUGINS_K4A_CONTEXT_K4A_IMG_POINT_CLOUD_TO_PCL_POINT_CLOUD_CONTEXT_T_HPP

#if DKSAVE_ENABLE_PCL

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <k4a/k4a.hpp>


namespace dksave::plugins_k4a
{

	/**
	 * 本类负责将 k4a:image 格式的点云图片转换为 pcl::PointCloud<pcl::PointXYZ> 格式
	 */
	struct k4a_img_point_cloud_to_pcl_point_cloud_context_t
	{
			pcl::PointCloud<pcl::PointXYZ> pcl;

			pcl::PointCloud<pcl::PointXYZ> & convert(const k4a::image & k4a_point_cloud)
			{
				pcl.clear();
				pcl.width = k4a_point_cloud.get_width_pixels();
				pcl.height = k4a_point_cloud.get_height_pixels();
				std::size_t size = pcl.width * pcl.height;
				pcl.resize(size);
				pcl.is_dense = false;
				const auto * k4a_point_cloud_buffer = reinterpret_cast<const std::int16_t *>(k4a_point_cloud.get_buffer());
				for (std::size_t i = 0; i < size; ++i) {
					pcl[i] = pcl::PointXYZ(
						k4a_point_cloud_buffer[3 * i + 0] / 1000.0f,
						k4a_point_cloud_buffer[3 * i + 1] / 1000.0f,
						k4a_point_cloud_buffer[3 * i + 2] / 1000.0f
					);
				}
				return this->pcl;
			}
	};

} // namespace dksave::plugins_k4a

#endif

#endif // DKSAVE_PLUGINS_K4A_CONTEXT_K4A_IMG_POINT_CLOUD_TO_PCL_POINT_CLOUD_CONTEXT_T_HPP
