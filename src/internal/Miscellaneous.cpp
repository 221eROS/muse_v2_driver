#include <muse_v2_driver/Miscellaneous.h>

void muse_v2_driver::Miscellaneous::setupInputCommands(ros::NodeHandle& node) {
	node.getParam("get_battery_charge", input_command.get_battery_charge);
	node.getParam("get_battery_voltage", input_command.get_battery_voltage);
}

bool muse_v2_driver::Miscellaneous::getBattery(Battery::Request& req, Battery::Response& res, MuseV2* muse_v2)
{
	bool out = false;

	ROS_INFO("request: get_battery_charge=%s, get_battery_voltage=%s", 
		req.get_battery_charge ? "true" : "false", 
		req.get_battery_voltage ? "true" : "false");

	if (req.get_battery_charge) {
		if (!received_command.get_battery_charge) {
			received_command.get_battery_charge = true;
			ROS_INFO("Asking Battery Charge.");
		}

		if (muse_v2->serial->getBatteryCharge() > -1) {
			res.battery_charge = muse_v2->serial->getBatteryCharge();
			out = true;
			ROS_INFO("Sending back battery charge (%%): %.2f", res.battery_charge);
		}
	}
	if (req.get_battery_voltage) {
		if (!received_command.get_battery_voltage) {
			received_command.get_battery_voltage = true;
			ROS_INFO("Asking Battery Voltage.");
		}
		if (muse_v2->serial->getBatteryVoltage() > -1) {
			res.battery_voltage = muse_v2->serial->getBatteryVoltage();
			out = true;
			ROS_INFO("Sending back battery voltage: %.2f", res.battery_voltage);
		}
	}

	return out;
}