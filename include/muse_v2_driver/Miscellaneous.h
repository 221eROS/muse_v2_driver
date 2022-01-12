#ifndef MISCELLANEOUS_H
#define MISCELLANEOUS_H

#include <muse_v2_driver/Muse.h>
#include <muse_v2_driver/Battery.h>

namespace muse_v2_driver {

	class Miscellaneous
	{
	public:

		struct CommandList
		{
			bool get_battery_charge = false;
			bool get_battery_voltage = false;

			bool operator==(const CommandList& rhs) const {
				return  (
					(get_battery_charge == rhs.get_battery_charge) &&
					(get_battery_voltage == rhs.get_battery_voltage)
					);
			}

		} default_command_list;

		CommandList input_command, received_command;

		Miscellaneous() = default;
		~Miscellaneous() = default;

		void setupInputCommands(ros::NodeHandle& node);
		bool getBattery(Battery::Request& req, Battery::Response& res, Muse* muse);

	};
}

#endif 