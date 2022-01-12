#include <muse_v2_driver/Configuration.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "get_configuration_params");

    ros::NodeHandle n;

    muse_v2_driver::Configuration config;

    config.setupInputCommands(n);

    ros::ServiceClient client = n.serviceClient<muse_v2_driver::GetConfigurationParams>("get_configuration_params");

    if (config.input_command == config.default_command_list) {
        ROS_ERROR("No request found.");
        ros::shutdown();
        return -1;
    }

    muse_v2_driver::GetConfigurationParams srv;

    if (config.input_command.get_gyroscope_full_scale)
        srv.request.get_gyroscope_full_scale = true;

    if (config.input_command.get_accelerometer_full_scale)
        srv.request.get_accelerometer_full_scale = true;

    if (config.input_command.get_accelerometer_hdr_full_scale)
        srv.request.get_accelerometer_hdr_full_scale = true;

    if (config.input_command.get_magnetometer_full_scale)
        srv.request.get_magnetometer_full_scale = true;

    if (config.input_command.get_log_mode)
        srv.request.get_log_mode = true;

    if (config.input_command.get_log_frequency)
        srv.request.get_log_frequency = true;

    if (client.call(srv))
    {
        if (srv.request.get_gyroscope_full_scale)
            ROS_INFO("Gyroscope Full-Scale [dps]: %u", srv.response.gyroscope_full_scale);

        if (srv.request.get_accelerometer_full_scale)
            ROS_INFO("Accelerometer Full-Scale [g]: %u", srv.response.accelerometer_full_scale);

        if (srv.request.get_accelerometer_hdr_full_scale)
            ROS_INFO("Accelerometer HDR Full-Scale [g]: %u", srv.response.accelerometer_hdr_full_scale);

        if (srv.request.get_magnetometer_full_scale)
            ROS_INFO("Magnetometer Full-Scale [G]: %u", srv.response.magnetometer_full_scale);

        if (srv.request.get_log_mode)
            ROS_INFO("Log Mode: %u", srv.response.log_mode);

        if (srv.request.get_log_frequency)
            ROS_INFO("Log Frequency: %u", srv.response.log_frequency);
    }
    else
    {
        ROS_ERROR("Failed to call Get Configuration Params service.");
        return 1;
    }

    return 0;
}