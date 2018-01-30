#include "param_utils.hpp"
#include "phidgets_wheels_publisher.h"

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

int main(int argc, char *argv[]) {
	ros::init(argc, argv, "phidgets_encoder_node");
	auto handle = ros::NodeHandle{ };
	auto private_handle = ros::NodeHandle{ "~" };

	const auto serial_number =
		umigv::get_parameter_default(private_handle, "serial_number", -1);
	const auto publish_rate =
		umigv::get_parameter_default(private_handle, "publish_rate", 60.0);
	const auto polling_rate =
		umigv::get_parameter_default(private_handle, "polling_rate", 60.0);

	ros::spin();
}
