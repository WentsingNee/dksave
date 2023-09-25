/**
 * @file       ucamera_factory.hpp
 * @brief
 * @date       2023-09-24
 * @author     Peter
 * @copyright
 *      Peter of [ThinkSpirit Laboratory](http://thinkspirit.org/)
 *   of [Nanjing University of Information Science & Technology](http://www.nuist.edu.cn/)
 *   all rights reserved
 */


#ifndef DKSAVE_UCAMERAFACTOR_HPP
#define DKSAVE_UCAMERAFACTOR_HPP

#include "ucamera.hpp"

#include <memory>
#include <vector>

#include <yaml-cpp/node/node.h>


class ucamera_factory
{
	public:
		virtual ~ucamera_factory() = default;

		virtual std::vector<std::unique_ptr<ucamera> >
		find_cameras(YAML::Node &yaml_config) = 0;
};

#endif //DKSAVE_UCAMERAFACTOR_HPP
