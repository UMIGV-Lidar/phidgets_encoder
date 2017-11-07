#include "ros/ros.h"
#include "phidgets_api/encoder.h"

#define PRINT(X) ROS_INFO_STREAM(#X << " := " << X)

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "encoder_test");
    ros::NodeHandle n;

    phidgets::Encoder encoder_board;
    encoder_board.open(-1);
    encoder_board.waitForAttachment(5000);

    PRINT(encoder_board.getDeviceLabel());
    PRINT(encoder_board.getDeviceName());
    PRINT(encoder_board.getDeviceSerialNumber());
    PRINT(encoder_board.getDeviceType());
    PRINT(encoder_board.getDeviceVersion());
    PRINT(encoder_board.getEncoderCount());
}
