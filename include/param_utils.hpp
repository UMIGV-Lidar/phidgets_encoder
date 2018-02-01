#include "ros_traits.hpp" // umigv::IsParameterV
#include "exceptions.h" // umigv::ParameterNotFoundException

#include <ros/ros.h> // ros::NodeHandle, ros::shutdown, ROS_FATAL_STREAM

#include <string> // std::string
#include <type_traits> // std::enable_if_t

namespace umigv {

template <typename T>
std::enable_if_t<IsParameterV<T>, T> get_parameter_fatal(
    const ros::NodeHandle &handle,
    const std::string &parameter
) {
    auto fetched = T{ };

    if (not handle.getParam(parameter, fetched)) {
        auto parameter_namespaced = handle.getNamespace() + "/" + parameter;
        throw ParameterNotFoundException{ "get_parameter_fatal",
                                          std::move(parameter_namespaced) };
    }

    return fetched;
}

template <typename T>
std::enable_if_t<IsParameterV<T>, T> get_parameter_default(
    const ros::NodeHandle &handle,
    const std::string &name,
    const T &fallback = T { }
) {
    auto fetched = T{ };
    handle.param(name, fetched, fallback);
    return fetched;
}

} // namespace umigv
