#include "ros/ros.h"
#include "phidgets_api/encoder.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_datatypes.h"
#include "ros_cpp11_range/range.hpp"

#include <boost/array.hpp>

#include <cstdint>
#include <cstddef>
#include <cstdlib>
#include <cmath>
#include <string>
#include <functional>
#include <vector>
#include <stdexcept>
#include <algorithm>
#include <tuple>
#include <utility>
#include <sstream>

static const constexpr std::size_t COVARIANCE_SIZE = 36;
static const constexpr std::size_t ZERO = 0;
static const constexpr std::size_t NUM_ENCODERS = 4;
using covariance_type = boost::array<double, COVARIANCE_SIZE>;

struct RobotState {
	double x, y, theta;
};

template <typename T>
T get_param_fatal(const ros::NodeHandle &node, const std::string &parameter) {
	T parameter_value;

	if (!node.getParam(parameter, parameter_value)) {
		std::ostringstream oss;
		oss << "count not retreive parameter " << node.getNamespace() << parameter;

		ROS_FATAL_STREAM(oss.str());
		throw std::runtime_error(oss.str());
	}

	return parameter_value;
}

geometry_msgs::Pose make_pose(const RobotState &state) {
	geometry_msgs::Pose pose;

	pose.position.x = state.x;
	pose.position.y = state.y;
	pose.position.z = 0;

	pose.orientation = tf::createQuaternionMsgFromYaw(state.theta);

	return pose;
}

geometry_msgs::Twist make_twist(
	const ros::NodeHandle &node,
	const RobotState &last,
	const RobotState &current
) {
	auto delta_t = get_param_fatal<double>(node, "polling_period");

	double delta_x = current.x - last.x;
	double delta_y = current.y - last.y;
	double delta_theta = current.theta - last.theta;

	geometry_msgs::Twist twist;

	twist.linear.x = delta_x / delta_t;
	twist.linear.y = delta_y / delta_t;
	twist.linear.z = 0.0;

	twist.angular.x = 0.0;
	twist.angular.y = 0.0;
	twist.angular.z = delta_theta / delta_t;

	return twist;
}

std::pair<int, double> initialize_parameters(const ros::NodeHandle &node) {
	using DVec = std::vector<double>;

	int serial_number;
	double polling_period;

	node.param("serial_number", serial_number, -1); // connect to any device
	node.param("polling_period", polling_period, 0.004); // 4ms polling period

	if (!node.hasParam("pose_covariance")) {
		node.setParam("pose_covariance", DVec(COVARIANCE_SIZE, 0.0));
	}

	if (!node.hasParam("twist_covariance")) {
		node.setParam("twist_covariance", DVec(COVARIANCE_SIZE, 0.0));
	}

	if (!node.hasParam("frame_id")) {
		node.setParam("frame_id", "phidgets_encoder");
	}

	return std::make_pair(serial_number, polling_period);
}

std::pair<covariance_type, covariance_type> get_covariance(
	const ros::NodeHandle &node
) {
	auto pose_covariance_vector =
		get_param_fatal<std::vector<double>>(node, "pose_covariance");
	auto twist_covariance_vector =
		get_param_fatal<std::vector<double>>(node, "twist_covariance");

	if (pose_covariance_vector.size() != COVARIANCE_SIZE) {
		std::ostringstream oss;
		oss << node.getNamespace() << "pose_covariance must have size " << COVARIANCE_SIZE;
		throw std::runtime_error(oss.str());
	}

	if (twist_covariance_vector.size() != COVARIANCE_SIZE) {
		std::ostringstream oss;
		oss << node.getNamespace() << "twist_covariance must have size " << COVARIANCE_SIZE;
		throw std::runtime_error(oss.str());
	}

	covariance_type pose_covariance, twist_covariance;

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

	return {pose_covariance, twist_covariance};
}

std::uint32_t get_sequence_id() {
	static std::uint32_t sequence_id = 0;

	return sequence_id++;
}

RobotState calculate_next_state(
	const ros::NodeHandle &node,
	phidgets::Encoder &encoder,
	RobotState &current,
	std::vector<int> &positions
) {
	auto track = get_param_fatal<double>(node, "track");
	auto meters_per_tick = get_param_fatal<double>(node, "meters_per_tick");

	std::vector<int> new_positions(NUM_ENCODERS);
	for (std::size_t i : util::lang::range(ZERO, NUM_ENCODERS)) {
		new_positions[i] = encoder.getPosition(i);
	}

	std::vector<double> delta_position(NUM_ENCODERS);
	std::transform(
		new_positions.cbegin(),
		new_positions.cend(),
		positions.cbegin(),
		delta_position.begin(),
		[meters_per_tick](int current, int last) {
			return (current - last) * meters_per_tick;
		}
	);

	double delta = std::accumulate(delta_position.cbegin(), delta_position.cend(), 0.0) / 4.0;
	double omega = (
		delta_position[3] - delta_position[2] +
		delta_position[1] - delta_position[0]
	) / (2.0 * track);

	RobotState next = {
		current.x + delta * std::cos(current.theta + omega / 2.0),
		current.y + delta * std::sin(current.theta + omega / 2.0),
		current.theta + omega
	};

	positions = std::move(new_positions);

	return next;
}

// channel 0 - front left
// channel 1 - front right
// channel 2 - back left
// channel 3 - back right

// L. C. Bento, U. Nunes, F. Moita and A. Surrecio, "Sensor fusion for precise autonomous vehicle navigation in outdoor semi-structured environments," Proceedings. 2005 IEEE Intelligent Transportation Systems, 2005., 2005, pp. 245-250.
void timer_callback(
	const ros::NodeHandle &node,
	const ros::Publisher &pub,
	phidgets::Encoder &encoder,
	RobotState &state,
	std::vector<int> &encoder_positions,
	const ros::TimerEvent&
) {
	if (static_cast<std::size_t>(encoder.getEncoderCount()) != NUM_ENCODERS) {
		ROS_FATAL_STREAM("must have four encoders connected");
		throw std::runtime_error("must have four encoders connected");
	}

	auto frame_id = get_param_fatal<std::string>(node, "frame_id");

	auto next_state = calculate_next_state(
		node,
		encoder,
		state,
		encoder_positions
	);

	// ROS developers: please switch to the STL at your earliest convenience
	covariance_type pose_covariance, twist_covariance;
	std::tie(pose_covariance, twist_covariance) = get_covariance(node);

	nav_msgs::Odometry odom;

	odom.header.seq = get_sequence_id();
	odom.header.stamp = ros::Time::now();
	odom.header.frame_id = frame_id;

	odom.pose.pose = make_pose(next_state);
	odom.pose.covariance = pose_covariance;

	odom.twist.twist = make_twist(node, state, next_state);
	odom.twist.covariance = twist_covariance;

	pub.publish(odom);

	state = std::move(next_state);
}

int main(int argc, char *argv[]) {
	ros::init(argc, argv, "phidgets_encoder_node");
	ros::NodeHandle public_node; // public namespace node
	ros::NodeHandle private_node("~"); // private namespace node

	int serial_number;
	double polling_period;

	std::tie(serial_number, polling_period) =
		initialize_parameters(private_node);

	phidgets::Encoder encoder;
	encoder.open(serial_number);

	if (static_cast<std::size_t>(encoder.getEncoderCount()) != NUM_ENCODERS) {
		ROS_FATAL_STREAM("must have four encoders connected");
		throw std::runtime_error("must have four encoders connected");
	}

	std::vector<int> encoder_positions(encoder.getEncoderCount());
	for (std::size_t i : util::lang::range(ZERO, NUM_ENCODERS)) {
		encoder.setEnabled(i, true);
		encoder_positions[i] = encoder.getPosition(i);
	}

	auto pub = public_node.advertise<nav_msgs::Odometry>("odom", 1000);

	RobotState state = {0.0, 0.0, 0.0};

	using std::placeholders::_1;
	auto timer = public_node.createTimer(
		ros::Duration(polling_period),
		std::bind(
			timer_callback,
			std::cref(private_node),
			std::cref(pub),
			std::ref(encoder),
			std::ref(state),
			std::ref(encoder_positions),
			_1
		)
	);

	ros::spin();

	encoder.close();
}