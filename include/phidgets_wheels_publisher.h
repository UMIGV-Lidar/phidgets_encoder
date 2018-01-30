#ifndef UMIGV_PHIDGETS_WHEELS_PUBLISHER_H
#define UMIGV_PHIDGETS_WHEELS_PUBLISHER_H

#include <phidgets_api/encoder.h> // phidgets::Encoder
#include <ros/ros.h> // ros::Publisher, ros::TimerEvent

#include <boost/circular_buffer.hpp> // boost::circular_buffer
#include <boost/container/static_vector.hpp> // boost::container::static_vector

#include <string> // std::string, std::to_string
#include <mutex> // std::mutex, std::lock_guard

namespace umigv {

enum class WheelCount {
    One,
    Two,
    Three,
    Four
};

class PhidgetsWheelsPublisher final : private phidgets::Encoder {
public:
    PhidgetsWheelsPublisher(int serial_number, ros::Publisher publisher,
                            std::string frame_id, WheelCount count,
                            double rads_per_tick);

    ~PhidgetsWheelsPublisher();

    void publish_state(const ros::TimerEvent &event) const;

    void poll_encoders(const ros::TimerEvent &event);

private:
    struct EncoderState;

    using BufferT = boost::circular_buffer<int>;
    using VectorT = boost::container::static_vector<EncoderState, 4>;

    void attachHandler() override;

    void detachHandler() override;

    void errorHandler(int error_code) override;

    void indexHandler(int, int) override;

    void positionChangeHandler(int index, int delta_time,
                               int delta_position) override;

    struct EncoderState {
        BufferT delta_positions{ 10 };
        BufferT delta_times{ 10 };
        int position;
        ros::Time time;
        mutable std::mutex mutex;
    };

    VectorT states_;
    ros::Publisher publisher_;
    const std::string frame_id_;
    const double rads_per_tick_;
};

} // namespace umigv

#endif