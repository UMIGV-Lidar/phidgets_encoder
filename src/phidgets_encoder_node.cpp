#include "encoder_state_publisher.h" // umigv::EncoderStatePublisher

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <umigv_utilities/types.hpp>
#include <umigv_utilities/utility.hpp> // umigv::blocking_shutdown
#include <umigv_utilities/exceptions.hpp> // umigv::ParameterNotFoundException,
										  // umigv::PhidgetException
#include <umigv_utilities/rosparam.hpp> // umigv:get_parameter_or,
										// umigv::get_parameter_fatal

#include <string> // operator""s
#include <tuple> // std::tuple

using namespace umigv::types;

struct Parameters {
	int serial_number = -1;
	f64 frequency;
	std::string frame_id = "encoders";
	f64 left_rads_per_tick;
	f64 right_rads_per_tick;
};

Parameters get_parameters(ros::NodeHandle &handle) {
	using namespace std::literals;

	Parameters params;

	params.serial_number = umigv::get_parameter_or(handle, "serial_number", -1);
	params.frequency = umigv::get_parameter_or(handle, "serial_number", 100.0);
	params.frame_id = umigv::get_parameter_or(handle, "frame_id", "encoders"s);
	params.left_rads_per_tick =
		umigv::get_parameter_fatal<f64>(handle, "left_rads_per_tick");
	params.right_rads_per_tick =
		umigv::get_parameter_fatal<f64>(handle, "right_rads_per_tick");

	return params;
}

std::tuple<int, f64, std::string>
get_defaultable_parameters(ros::NodeHandle &handle) {
	using namespace std::literals;

	return std::tuple<int, f64, std::string>{
		umigv::get_parameter_or(handle, "serial_number", -1),
		umigv::get_parameter_or(handle, "frequency", 100.0),
		umigv::get_parameter_or(handle, "frame_id", "encoders"s)
	};
}

int main(int argc, char *argv[]) {
	ros::init(argc, argv, "phidgets_encoder_node");
	ros::NodeHandle handle;
	ros::NodeHandle private_handle{ "~" };

	Parameters params = [&handle] {
		try {
			return get_parameters(handle);
		} catch (const umigv::ParameterNotFoundException &e) {
			ROS_FATAL_STREAM("parameter '" << e.parameter() << "' not found");

			umigv::blocking_shutdown();
		}
	}();

	try {
		umigv::EncoderStatePublisher state_publisher{
			handle.advertise<sensor_msgs::JointState>("wheel_state", 10),
			umigv::RobotCharacteristics{ }
				.with_frame(std::move(params.frame_id))
				.with_rads_per_tick(params.left_rads_per_tick,
				                    params.right_rads_per_tick)
				.with_serial_number(params.serial_number)
		};

		auto publish_timer =
			handle.createTimer(ros::Rate(params.frequency),
			                   &umigv::EncoderStatePublisher::publish_state,
			                   &state_publisher);

		ros::spin();
	} catch (const umigv::PhidgetsException &e) {
		ROS_FATAL_STREAM(e.what() << ": " << e.error_description() << " ("
						 << e.error_code() << ")");

		umigv::blocking_shutdown();
	}
}
