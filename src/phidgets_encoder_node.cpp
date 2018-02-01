#include "exceptions.h"
#include "param_utils.hpp"
#include "phidgets_wheels_publisher.h"

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include <cstdlib> // std::exit, EXIT_FAILURE
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
		umigv::get_parameter_default(private_handle, "polling_rate", 60.0);
	auto frame_id =
		umigv::get_parameter_default(private_handle, "frame_id", "encoders"s);
	auto buffer_length_param =
		umigv::get_parameter_default(private_handle, "buffer_length", 10);

	if (buffer_length_param <= 0) {
		ROS_WARN_STREAM("invalid ~buffer_length: " << buffer_length_param
		                << ", using default value of 10...");
		buffer_length_param = 10;
	}

	const auto buffer_length = static_cast<std::size_t>(buffer_length_param);

	auto wheel_count = int{ };
	double rads_per_tick = double{ };

	try {
		wheel_count = umigv::get_parameter_fatal<int>(private_handle,
		                                              "wheel_count");
		rads_per_tick = umigv::get_parameter_fatal<double>(private_handle,
		                                                   "rads_per_tick");
	} catch (const umigv::ParameterNotFoundException &e) {
		ROS_FATAL_STREAM("unable to find parameter " << e.parameter());
		ros::shutdown();
		ros::waitForShutdown();
		std::exit(EXIT_FAILURE);
	} 

	if (wheel_count < 1 or wheel_count > 4) {
		ROS_FATAL_STREAM("invalid ~wheel_count: " << wheel_count);
		ros::shutdown();
		ros::waitForShutdown();
		std::exit(EXIT_FAILURE);
	}

	const auto wheel_count_enum =
		static_cast<umigv::WheelCount>(wheel_count - 1);

	auto state_publisher =
		handle.advertise<sensor_msgs::JointState>("wheel_state", 10);

	try {
		auto encoder_publisher =
			umigv::PhidgetsWheelsPublisher{ serial_number,
											std::move(state_publisher),
											std::move(frame_id),
											wheel_count_enum,
											rads_per_tick, buffer_length };

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
	} catch (const umigv::UnableToConnectException &e) {
		ROS_FATAL_STREAM("unable to connect: " << e.code() << ": " << e.what());
		ros::shutdown();
		ros::waitForShutdown();
		std::exit(EXIT_FAILURE);
	} catch (const umigv::DeviceDetachedException &e) {
		ROS_FATAL_STREAM("device detached");
		ros::shutdown();
		ros::waitForShutdown();
		std::exit(EXIT_FAILURE);
	}
}
