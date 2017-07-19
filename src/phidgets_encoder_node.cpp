#include "ros/ros.h"
#include "phidgets_api/encoder.h"
#include "nav_msgs/Odometry.h"

#include <boost/array.hpp>

#include <cstdint>
#include <string>
#include <functional>
#include <vector>
#include <stdexcept>
#include <algorithm>

// channel 0 - front left
// channel 1 - front right
// channel 2 - back left
// channel 3 - back right
void timer_callback(
	ros::NodeHandle &n,
	ros::Publisher &pub,
	phidgets::Encoder &encoder,
	const ros::TimerEvent&
) {
	static std::uint32_t sequence_id = 0;

	int num_encoders = encoder.getEncoderCount(); // should be std::size_t

	std::vector<double> pose_covariance_vector, twist_covariance_vector;
	std::string frame_id;

	n.getParam("pose_covariance", pose_covariance_vector);
	n.getParam("twist_covariance", twist_covariance_vector);
	n.getParam("frame_id", frame_id);

	if (
		(pose_covariance_vector.size() != 36) ||
		(twist_covariance_vector.size() != 36)
	) {
		throw std::runtime_error("timer_callback");
	}

	// ROS developers: please switch to the STL at your earliest convenience
	boost::array<double, 36> pose_covariance, twist_covariance;

	std::move(
		pose_covariance_vector.begin(),
		pose_covariance_vector.end(),
		pose_covariance.begin()
	);
	std::move(
		twist_covariance_vector.begin(),
		twist_covariance_vector.end(),
		twist_covariance.begin()
	);

	nav_msgs::Odometry odom;
	odom.header.seq = sequence_id;
	odom.header.stamp = ros::Time::now();
	odom.header.frame_id = frame_id;
	odom.pose.covariance = pose_covariance;
	odom.twist.covariance = twist_covariance;

	pub.publish(odom);

	++sequence_id;
}

int main(int argc, char *argv[]) {
	ros::init(argc, argv, "phidgets_encoder_node");
	ros::NodeHandle public_node; // public namespace node
	ros::NodeHandle private_node("~"); // private namespace node

	int serial_number = -1; // connect to any device
	double polling_rate = 0.004; // 4ms polling rate

	if (!private_node.hasParam("serial_number")) {
		private_node.setParam("serial_number", serial_number);
	} else {
		private_node.getParam("serial_number", serial_number);
	}

	if (!private_node.hasParam("polling_rate")) {
		private_node.setParam("polling_rate", polling_rate);
	} else {
		private_node.getParam("polling_rate", polling_rate);
	}

	if (!private_node.hasParam("pose_covariance")) {
		private_node.setParam("pose_covariance", std::vector<double>(36, 0.0));
	}

	if (!private_node.hasParam("twist_covariance")) {
		private_node.setParam(
			"twist_covariance",
			std::vector<double>(36, 0.0)
		);
	}

	if (!private_node.hasParam("frame_id")) {
		private_node.setParam("frame_id", "phidgets_encoder");
	}

	auto pub = public_node.advertise<nav_msgs::Odometry>("odom", 1000);

	phidgets::Encoder encoder;
	encoder.open(serial_number);

	// enable all encoders
	for (int i = 0; i < encoder.getEncoderCount(); ++i) {
		encoder.setEnabled(i, true);
	}

	using std::placeholders::_1;
	auto timer = public_node.createTimer(
		ros::Duration(polling_rate),
		std::bind(
			timer_callback,
			std::ref(private_node),
			std::ref(pub),
			std::ref(encoder),
			_1
		)
	);

	ros::spin();

	encoder.close();
}