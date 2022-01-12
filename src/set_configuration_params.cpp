#include <muse_v2_driver/Configuration.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "set_configuration_params");

	ros::NodeHandle n;

	muse_v2_driver::Configuration config;

	config.setupInputCommands(n);

	ros::ServiceClient client = n.serviceClient<muse_v2_driver::SetConfigurationParams>("set_configuration_params");

	if (config.input_command == config.default_command_list) {
		ROS_ERROR("No request found.");
		ros::shutdown();
		return -1;
	}

	muse_v2_driver::SetConfigurationParams srv;

	if (config.input_command.gyroscope_full_scale != 0) {
		srv.request.set_gyroscope_full_scale = true;
		srv.request.gyroscope_full_scale = config.input_command.gyroscope_full_scale;
	}

	if (config.input_command.accelerometer_full_scale != 0) {
		srv.request.set_accelerometer_full_scale = true;
		srv.request.accelerometer_full_scale = config.input_command.accelerometer_full_scale;
	}

	if (config.input_command.accelerometer_hdr_full_scale != 0) {
		srv.request.set_accelerometer_hdr_full_scale = true;
		srv.request.accelerometer_hdr_full_scale = config.input_command.accelerometer_hdr_full_scale;
	}

	if (config.input_command.magnetometer_full_scale != 0) {
		srv.request.set_magnetometer_full_scale = true;
		srv.request.magnetometer_full_scale = config.input_command.magnetometer_full_scale;
	}
	
	if (config.input_command.log_mode != UINT8_MAX) {
		srv.request.set_log_mode = true;
		srv.request.log_mode = config.input_command.log_mode;
	}
	
	if (config.input_command.log_frequency != UINT8_MAX) {
		srv.request.set_log_frequency = true;
		srv.request.log_frequency = config.input_command.log_frequency;
	}
	
	if (client.call(srv))
	{
		if (srv.request.set_gyroscope_full_scale)
			ROS_INFO("Gyroscope Full-Scale value changed: %s", srv.response.gyroscope_full_scale_changed ? "true" : "false");

		if (srv.request.set_accelerometer_full_scale)
			ROS_INFO("Accelerometer Full-Scale value changed: %s", srv.response.accelerometer_full_scale_changed ? "true" : "false");

		if (srv.request.set_accelerometer_hdr_full_scale)
			ROS_INFO("Accelerometer HDR Full-Scale value changed: %s", srv.response.accelerometer_hdr_full_scale_changed ? "true" : "false");

		if (srv.request.set_magnetometer_full_scale)
			ROS_INFO("Magnetometer Full-Scale value changed: %s", srv.response.magnetometer_full_scale_changed ? "true" : "false");

		if (srv.request.set_log_mode)
			ROS_INFO("Log Mode changed: %s", srv.response.log_mode_changed ? "true" : "false");

		if (srv.request.set_log_frequency)
			ROS_INFO("Log Frequency changed: %s", srv.response.log_frequency_changed ? "true" : "false");
	}
	else
	{
		ROS_ERROR("Failed to call Set Configuration Params service.");
		return 1;
	}

	return 0;
}