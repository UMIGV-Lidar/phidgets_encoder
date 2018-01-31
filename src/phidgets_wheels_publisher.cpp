#include "phidgets_wheels_publisher.h"

#include <sensor_msgs/JointState.h> // sensor_msgs::JointState

#include <cmath> // std::round
#include <cstdint> // std::uint32_t
#include <stdexcept> // std::runtime_error

umigv::PhidgetsWheelsPublisher::PhidgetsWheelsPublisher(
    const int serial_number, ros::Publisher publisher,
    std::string frame_id, const WheelCount count, const double rads_per_tick
) : phidgets::Encoder{ }, states_{ static_cast<VectorT::size_type>(count) + 1 },
    publisher_{ std::move(publisher) }, frame_id_{ std::move(frame_id) },
    rads_per_tick_{ rads_per_tick }
{
    phidgets::Encoder::open(serial_number);

    if (serial_number == -1) {
        ROS_INFO_STREAM("trying to connect to any device...");
    } else {
        ROS_INFO_STREAM(
            "trying to connect to device with serial number "
            << serial_number << "..."
        );
    }

    for (auto i = 0; i < 10; ++i) {
        const auto result = phidgets::Phidget::waitForAttachment(1000);

        if (result == 0) {
            break;
        }

        if (i < 9) {
            ROS_WARN_STREAM("unable to connect, retrying...");
        } else {
            throw std::runtime_error{
                "PhidgetsWheelsPublisher::PhidgetsWheelsPublisher"
            };
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

    work_thread_ = std::thread([this] { process_queue(); });
}

umigv::PhidgetsWheelsPublisher::~PhidgetsWheelsPublisher() {
    phidgets::Phidget::close();

    destructing_ = true;
    work_thread_.join();
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
        auto dt = 0.0;

        {

        auto &state = states_[i];
        std::lock_guard<std::mutex> guard{ state.mutex };

        to_publish.position.push_back(rads_per_tick_
                                      * static_cast<double>(state.position));

        dr = std::accumulate(state.delta_positions.begin(),
                             state.delta_positions.end(),
                             0.0) * rads_per_tick_;
        dt = std::accumulate(state.delta_times.begin(),
                             state.delta_times.end(),
                             0.0) * 1e-3;

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
    ROS_FATAL_STREAM("encoder board detached, shutting down...");
    throw std::runtime_error{ "PhidgetsWheelsPublisher::detachHandler" };
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

    std::lock_guard<std::mutex> guard{ queue_mutex_ };
    work_queue_.emplace(std::async(
        &PhidgetsWheelsPublisher::position_changed_impl, this,
        index, now, delta_position
    ));
}

void umigv::PhidgetsWheelsPublisher::position_changed_impl(
    const int index, const ros::Time now,
    const int delta_position
) {
    auto &state = states_[index];
    std::lock_guard<std::mutex> guard{ state.mutex };

    const auto dt = now - state.time;
    const auto dt_ms = static_cast<int>(std::round(dt.toSec() * 1.0e3));

    state.delta_positions.push_back(delta_position);
    state.delta_times.push_back(dt_ms);
    state.time = now;
    state.position += delta_position;
    state.updated = true;
}

void umigv::PhidgetsWheelsPublisher::process_queue() {
    while (not destructing_) {
        // redo this with condvars?
        std::lock_guard<std::mutex> guard{ queue_mutex_ };

        if (destructing_) {
            return;
        }

        if (not work_queue_.empty()) {
            work_queue_.front().get();
            work_queue_.pop();
        }
    }
}

umigv::PhidgetsWheelsPublisher::EncoderState::EncoderState(
    const EncoderState &other
) {
    std::lock_guard<std::mutex> guard{ other.mutex };

    delta_positions = other.delta_positions;
    delta_times = other.delta_times;
    time = other.time;
    position = other.position;
    updated = other.updated;
}

umigv::PhidgetsWheelsPublisher::EncoderState::EncoderState(
    EncoderState &&other
) noexcept {
    std::lock_guard<std::mutex> guard{ other.mutex };

    delta_positions = std::move(other.delta_positions);
    delta_times = std::move(other.delta_times);
    time = other.time;
    position = other.position;
    updated = other.updated;
}
