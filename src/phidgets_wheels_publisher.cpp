#include "phidgets_wheels_publisher.h"

#include <sensor_msgs/JointState.h> // sensor_msgs::JointState

#include <cmath> // std::round
#include <cstdint> // std::uint32_t
#include <atomic> // std::atomic
#include <stdexcept> // std::runtime_error
#include <string> // operator ""s

umigv::PhidgetsWheelsPublisher::PhidgetsWheelsPublisher(
    const int serial_number, ros::Publisher publisher,
    std::string frame_id, const WheelCount count, const double rads_per_tick,
    const std::size_t buffer_length
) : phidgets::Encoder{ },
    states_(static_cast<VectorT::size_type>(count) + 1,
            EncoderState{ buffer_length } ),
    publisher_{ std::move(publisher) }, frame_id_{ std::move(frame_id) },
    rads_per_tick_{ rads_per_tick }
{
    using namespace std::literals;

    phidgets::Encoder::open(serial_number);

    if (serial_number == -1) {
        ROS_INFO_STREAM("trying to connect to any device...");
    } else {
        ROS_INFO_STREAM(
            "trying to connect to device with serial number "
            << serial_number << "..."
        );
    }

    constexpr auto TRIES = 3;
    for (auto i = 0; i < TRIES; ++i) {
        const auto result = phidgets::Phidget::waitForAttachment(2000);

        if (result == 0) {
            break;
        }

        if (i < TRIES - 1) {
            ROS_WARN_STREAM("unable to connect, retrying...");
        } else {
            const auto description =
                "PhidgetsWheelsPublisher::PhidgetsWheelsPublisher: "s
                + phidgets::Phidget::getErrorDescription(result);

            throw UnableToConnectException{ result, description };
        }
    }

    const auto name = phidgets::Phidget::getDeviceName();
    const auto sn = phidgets::Phidget::getDeviceSerialNumber();

    ROS_INFO_STREAM("connected to " << name << " with serial number " << sn);

    const auto num_inputs = static_cast<int>(count) + 1;

    for (auto i = 0; i < num_inputs; ++i) {
        phidgets::Encoder::setEnabled(i, true);

        auto &state = states_[i];
        std::lock_guard<std::mutex> guard{ state.mutex };
        state.position = phidgets::Encoder::getPosition(i);
    }
}

umigv::PhidgetsWheelsPublisher::~PhidgetsWheelsPublisher() {
    phidgets::Phidget::close();
}

void umigv::PhidgetsWheelsPublisher::publish_state(
    const ros::TimerEvent &event
) const {
    using namespace std::literals;

    static std::atomic<std::uint32_t> sequence_id{ 0 };

    const auto to_publish_ptr =
        sensor_msgs::JointState::Ptr{ new sensor_msgs::JointState };
    auto &to_publish = *to_publish_ptr;

    to_publish.header.seq = sequence_id++;
    to_publish.header.stamp = event.current_real;
    to_publish.header.frame_id = frame_id_;

    for (auto i = VectorT::size_type{ 0 }; i < states_.size(); ++i) {
        to_publish.name.push_back("wheel"s + std::to_string(i));

        auto dr = 0.0;
        auto dt = 1.0;

        {

        auto &state = states_[i];
        std::lock_guard<std::mutex> guard{ state.mutex };

        to_publish.position.push_back(rads_per_tick_
                                      * static_cast<double>(state.position));

        if (not state.delta_positions.empty()
            and not state.delta_times.empty()) {
            dr = std::accumulate(state.delta_positions.begin(),
                                 state.delta_positions.end(),
                                 0.0) * rads_per_tick_;
            dt = std::accumulate(state.delta_times.begin(),
                                 state.delta_times.end(),
                                 ros::Duration{ 0 }).toSec();;
        }

        }

        const auto vbar = dr / dt;

        to_publish.velocity.push_back(vbar);
    }

    publisher_.publish(to_publish_ptr);
}

void umigv::PhidgetsWheelsPublisher::poll_encoders(const ros::TimerEvent&) {
    for (auto i = VectorT::size_type{ 0 }; i < states_.size(); ++i) {
        auto &state = states_[i];
        std::lock_guard<std::mutex> guard{ state.mutex };

        if (state.updated) {
            state.updated = false;
        } else {
            state.delta_positions.clear();
            state.delta_times.clear();
        }
    }
}

void umigv::PhidgetsWheelsPublisher::attachHandler() { }

void umigv::PhidgetsWheelsPublisher::detachHandler() {
    throw DeviceDetachedException{ "PhidgetsWheelsPublisher::detachHandler" };
}

void umigv::PhidgetsWheelsPublisher::errorHandler(const int error_code) {
    const auto error = phidgets::Phidget::getErrorDescription(error_code);

    ROS_ERROR_STREAM("error " << error_code << ": " << error);
}

void umigv::PhidgetsWheelsPublisher::indexHandler(int, int) { }

void umigv::PhidgetsWheelsPublisher::positionChangeHandler(
    const int index, int, const int delta_position
) {
    const auto now = ros::Time::now();

    auto &state = states_[index];
    std::lock_guard<std::mutex> guard{ state.mutex };

    const auto dt = now - state.time;

    state.delta_times.push_back(dt);
    state.delta_positions.push_back(delta_position);
    state.time = now;
    state.position += delta_position;
    state.updated = true;
}

umigv::PhidgetsWheelsPublisher::EncoderState::EncoderState(
    const std::size_t buffer_length
) : delta_times{ buffer_length }, delta_positions{ buffer_length }, mutex{ },
    time{ }, position{ }, updated{ false }
{ }

umigv::PhidgetsWheelsPublisher::EncoderState::EncoderState(
    const EncoderState &other
) {
    std::lock_guard<std::mutex> this_guard{ mutex };
    std::lock_guard<std::mutex> other_guard{ other.mutex };

    delta_times = other.delta_times;
    delta_positions = other.delta_positions;
    position = other.position;
    time = other.time;
}

umigv::PhidgetsWheelsPublisher::EncoderState::EncoderState(
    EncoderState &&other
) noexcept {
    std::lock_guard<std::mutex> this_guard{ mutex };
    std::lock_guard<std::mutex> other_guard{ other.mutex };

    delta_times = std::move(other.delta_times);
    delta_positions = std::move(other.delta_positions);
    position = other.position;
    time = other.time;
}
