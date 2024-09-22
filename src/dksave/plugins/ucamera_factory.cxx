/**
 * @file       ucamera_factory.cxx
 * @brief
 * @date       2023-09-24
 * @author     Wentsing Nee
 * @copyright
 *      Wentsing Nee of China Agricultural University
 *   all rights reserved
 */

module;

#include <yaml-cpp/node/node.h>

#include <kerbal/container/vector.hpp>
#include <kerbal/utility/tuple.hpp>

export module dksave.plugins.ucamera_factory;

export import dksave.plugins.ucamera;


namespace dksave
{

	export
	template <typename Factory>
	concept ucamera_factory = requires (YAML::Node const & yaml_config)
	{
		{ Factory::find_cameras(yaml_config) };
	};


	export
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
