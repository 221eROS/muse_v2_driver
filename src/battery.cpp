#include <muse_v2_driver/Miscellaneous.h>
#include <muse_v2_driver/Battery.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "battery");

    ros::NodeHandle n;

    muse_v2_driver::Miscellaneous misc;

    misc.setupInputCommands(n);

    ros::ServiceClient client = n.serviceClient<muse_v2_driver::Battery>("battery");

    if (misc.input_command == misc.default_command_list) {
        ROS_ERROR("No request found.");
        ros::shutdown();
        return -1;
    }

    muse_v2_driver::Battery srv;

    if (misc.input_command.get_battery_charge) 
        srv.request.get_battery_charge = true;

    if (misc.input_command.get_battery_voltage)
        srv.request.get_battery_voltage = true;

    if (client.call(srv))
    {
        if (srv.request.get_battery_charge) 
            ROS_INFO("Battery charge (%%): %.2f", srv.response.battery_charge);

        if (srv.request.get_battery_voltage) 
            ROS_INFO("Battery voltage: %.2f", srv.response.battery_voltage);
    }
    else
    {
        ROS_ERROR("Failed to call battery service.");
        return 1;
    }

    return 0;
}