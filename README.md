## Synopsis

`phidgets_encoder` interfaces with a Phidgets 1047 High Speed Encoder board and outputs a JointState for as many wheels as specified, including average rotational velocity averaged over the last few position change events.

## Code Example

	rosrun phidgets_encoder phidgets_encoder_node _wheel_count:=2 _rads_per_tick:=0.00436332313 _serial_number:=-1 _publish_rate:=60.0 _polling_rate:=60.0 _frame_id:=encoders _buffer_length:=10

## Motivation

Getting a JointState for each encoder-bound wheel is the first step to generating Odometry data from encoder data. Combining the JointState messages with data from a URDF will allow us to broadcast Odometry messages based on encoder data.

## Installation

Clone this repository into your `catkin` workspace, then run `catkin_make`. Optionally, run `catkin_make install` to install the package to your `catkin` workspace.

## Contributors

`phidgets_encoder` is authored and maintained by Gregory Meyer of UMIGV.
