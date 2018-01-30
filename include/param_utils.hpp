#include "ros_traits.hpp" // umigv::IsParameterV

#include <ros/ros.h> // ros::NodeHandle, ros::shutdown, ROS_FATAL_STREAM

#include <string> // std::string
#include <type_traits> // std::enable_if_t

namespace umigv {

template <typename T>
std::enable_if_t<IsParameterV<T>, T> get_parameter_fatal(
    const ros::NodeHandle &node,
    const std::string &parameter
) {
    T parameter_value;

    if (not node.getParam(parameter, parameter_value)) {
        const char *const err = "get_param_fatal: could not retreive parameter ";

        ROS_FATAL_STREAM(err << node.getNamespace() << "/" << parameter);
        ros::shutdown();
    }

    return parameter_value;
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
