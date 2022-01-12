#include <muse_v2_driver/Stream.h>
#include <muse_v2_driver/Transmission.h>

const int DEFAULT_QUEUE_SIZE = 10;
const int DEFAULT_FREQUENCY = 50;

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "start_transmission");

	ros::NodeHandle n;

	int publisher_queue_size = DEFAULT_QUEUE_SIZE;
	int rate = DEFAULT_FREQUENCY;
	int frequency = DEFAULT_FREQUENCY;

	if (!n.getParam("publisher_queue_size", publisher_queue_size))
		ROS_WARN("No publisher_queue_size parameter found on server. Use default one.");

	if (!n.getParam("rate", rate))
		ROS_WARN("No rate parameter found on server. Use default one.");

	if (!n.getParam("frequency", frequency))
		ROS_WARN("No frequency parameter found on server. Use default one.");

	muse_v2_driver::Stream stream;
	stream.setupInputCommands(n);

	if (!stream.isFrequencyAdmissible(frequency)) {
		ROS_ERROR("The admitted acquisition frequency in STREAM mode range between 1 and 150Hz. Transmission stopped.");
		ros::shutdown();
		return -1;
	}

	if (stream.input_command == stream.default_command_list) {
		ROS_ERROR("No transmission request found. Transmission stopped.");
		ros::shutdown();
		return -1;
	}

	ros::Publisher command_pub = n.advertise<muse_v2_driver::Transmission>("start_transmission", publisher_queue_size);

	muse_v2_driver::Transmission transmission_msg;
	transmission_msg.frequency = frequency;

	ros::Rate loop_rate(rate);

	while (ros::ok())
	{
		if (stream.input_command.get_imu) {
			transmission_msg.transmission.data.push_back(static_cast<int>(muse_v2_driver::Stream::CommandType::GET_IMU));
		}

		if (stream.input_command.get_quaternion) {
			transmission_msg.transmission.data.push_back(static_cast<int>(muse_v2_driver::Stream::CommandType::GET_QUATERNION));
		}

		if (stream.input_command.get_angular_velocity) {
			transmission_msg.transmission.data.push_back(static_cast<int>(muse_v2_driver::Stream::CommandType::GET_ANGULAR_VELOCITY));
		}

		if (stream.input_command.get_acceleration) {
			transmission_msg.transmission.data.push_back(static_cast<int>(muse_v2_driver::Stream::CommandType::GET_ACCELERATION));
		}

		if (stream.input_command.get_mag) {
			transmission_msg.transmission.data.push_back(static_cast<int>(muse_v2_driver::Stream::CommandType::GET_MAG));
		}

		if (stream.input_command.get_rpy) {
			transmission_msg.transmission.data.push_back(static_cast<int>(muse_v2_driver::Stream::CommandType::GET_RPY));
		}
			
		command_pub.publish(transmission_msg);

		ros::spinOnce();

		loop_rate.sleep();
	}
 
	return 0;
}