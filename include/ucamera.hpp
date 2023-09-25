/**
 * @file       ucamera.hpp
 * @brief
 * @date       2023-09-24
 * @author     Peter
 * @copyright
 *      Peter of [ThinkSpirit Laboratory](http://thinkspirit.org/)
 *   of [Nanjing University of Information Science & Technology](http://www.nuist.edu.cn/)
 *   all rights reserved
 */

#ifndef DKSAVE_UCAMERA_HPP
#define DKSAVE_UCAMERA_HPP

#include <string>
#include <filesystem>


class ucamera {

protected:

    std::string k_device_name;
    bool k_enable;

public:

    ucamera(std::string const &device_name) :
            k_device_name(device_name),
            k_enable(false) {
    }

    virtual ~ucamera() = default;

    std::string const & device_name() const
    {
        return k_device_name;
    }

    bool enable() const
    {
        return k_enable;
    }

    virtual void start() = 0;
    virtual void stabilize() = 0;
    virtual void stop() noexcept = 0;

	virtual void working_loop(std::filesystem::path const & working_dir, std::chrono::milliseconds sleep_period) = 0;

};

#endif //DKSAVE_UCAMERA_HPP
