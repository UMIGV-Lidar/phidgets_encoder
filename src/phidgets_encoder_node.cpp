#include "param_utils.hpp"
#include "phidgets_wheels_publisher.h"

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include <string> // operator""s
#include <thread> // std::thread::hardware_concurrency

int main(int argc, char *argv[]) {
	using namespace std::literals;

	ros::init(argc, argv, "phidgets_encoder_node");
	auto handle = ros::NodeHandle{ };
	auto private_handle = ros::NodeHandle{ "~" };

	const auto serial_number =
		umigv::get_parameter_default(private_handle, "serial_number", -1);
	const auto publish_rate =
		umigv::get_parameter_default(private_handle, "publish_rate", 60.0);
	const auto polling_rate =
		umigv::get_parameter_default(private_handle, "polling_rate", 10.0);
	auto frame_id =
		umigv::get_parameter_default(private_handle, "frame_id", "encoders"s);

	const auto wheel_count =
		umigv::get_parameter_fatal<int>(private_handle, "wheel_count");
	const auto rads_per_tick =
		umigv::get_parameter_fatal<double>(private_handle, "rads_per_tick");

	if (wheel_count < 1 or wheel_count > 4) {
		ROS_FATAL_STREAM("invalid ~wheel_count: " << wheel_count);
		ros::shutdown();
	}

	const auto wheel_count_enum =
		static_cast<umigv::WheelCount>(wheel_count - 1);

	auto state_publisher =
		handle.advertise<sensor_msgs::JointState>("wheel_state", 10);

	umigv::PhidgetsWheelsPublisher encoder_publisher{
		serial_number, std::move(state_publisher), std::move(frame_id),
		wheel_count_enum, rads_per_tick
	};

	auto publish_timer =
		handle.createTimer(ros::Rate(publish_rate),
		                   &umigv::PhidgetsWheelsPublisher::publish_state,
		                   &encoder_publisher);
	auto poll_timer =
		handle.createTimer(ros::Rate(polling_rate),
		                   &umigv::PhidgetsWheelsPublisher::poll_encoders,
		                   &encoder_publisher);

	auto spinner = ros::AsyncSpinner{ std::thread::hardware_concurrency() };
	spinner.start();
	ros::waitForShutdown();
}
