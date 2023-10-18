/**
 * @file       ucamera_factory.hpp
 * @brief
 * @date       2023-09-24
 * @author     Wentsing Nee
 * @copyright
 *      Wentsing Nee of China Agricultural University
 *   all rights reserved
 */


#ifndef DKSAVE_UCAMERAFACTOR_HPP
#define DKSAVE_UCAMERAFACTOR_HPP

#include "ucamera.hpp"

#include <yaml-cpp/node/node.h>

#include <kerbal/container/vector.hpp>
#include <kerbal/utility/tuple.hpp>


template <typename Factory>
concept ucamera_factory = requires (YAML::Node const &yaml_config) {
	{ Factory::find_cameras(yaml_config) };
};


template <int _, ucamera_factory ... Factory>
struct ucamera_factories :
		kerbal::utility::tuple<Factory...>
{
	private:
		using super = kerbal::utility::tuple<Factory...>;

	public:
		ucamera_factories() = default;
};

#endif // DKSAVE_UCAMERAFACTOR_HPP
