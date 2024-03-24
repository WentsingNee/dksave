/**
 * @file       ucamera_factory.hpp
 * @brief
 * @date       2023-09-24
 * @author     Wentsing Nee
 * @copyright
 *      Wentsing Nee of China Agricultural University
 *   all rights reserved
 */

#ifndef DKSAVE_PLUGINS_UCAMERA_FACTORY_HPP
#define DKSAVE_PLUGINS_UCAMERA_FACTORY_HPP

#include "ucamera.hpp"

#include <yaml-cpp/node/node.h>

#include <kerbal/container/vector.hpp>
#include <kerbal/utility/tuple.hpp>


namespace dksave
{

	template <typename Factory>
	concept ucamera_factory = requires (YAML::Node const & yaml_config)
	{
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

} // namespace dksave

#endif // DKSAVE_PLUGINS_UCAMERA_FACTORY_HPP
