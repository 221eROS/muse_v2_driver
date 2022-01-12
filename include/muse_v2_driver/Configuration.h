#ifndef CONFIGURATION_H
#define CONFIGURATION_H

#include <muse_v2_driver/Muse.h>
#include <muse_v2_driver/GetConfigurationParams.h>
#include <muse_v2_driver/SetConfigurationParams.h>

namespace muse_v2_driver {

	class Configuration
	{
	public:

		struct CommandList
		{
			bool get_gyroscope_full_scale = false;
			bool get_accelerometer_full_scale = false;
			bool get_accelerometer_hdr_full_scale = false;
			bool get_magnetometer_full_scale = false;
			bool get_log_mode = false;
			bool get_log_frequency = false;

			uint16_t gyroscope_full_scale = 0;
			uint8_t accelerometer_full_scale = 0;
			uint16_t accelerometer_hdr_full_scale = 0;
			uint8_t magnetometer_full_scale = 0;
			uint8_t log_mode = UINT8_MAX;
			uint8_t log_frequency = UINT8_MAX;

			bool operator==(const CommandList& rhs) const {
				return  (
					(get_gyroscope_full_scale == rhs.get_gyroscope_full_scale) &&
					(get_accelerometer_full_scale == rhs.get_accelerometer_full_scale) &&
					(get_accelerometer_hdr_full_scale == rhs.get_accelerometer_hdr_full_scale) &&
					(get_magnetometer_full_scale == rhs.get_magnetometer_full_scale) &&
					(get_log_mode == rhs.get_log_mode) &&
					(gyroscope_full_scale == rhs.gyroscope_full_scale) &&
					(accelerometer_full_scale == rhs.accelerometer_full_scale)&&
					(accelerometer_hdr_full_scale == rhs.accelerometer_hdr_full_scale)&&
					(magnetometer_full_scale == rhs.magnetometer_full_scale)&&
					(log_mode == rhs.log_mode)&&
					(log_frequency == rhs.log_frequency)
					);
			}

		} default_command_list;

		CommandList input_command, received_command;

		Configuration() = default;
		~Configuration() = default;

		void setupInputCommands(ros::NodeHandle& node);
		bool getConfigurationParams(GetConfigurationParams::Request& req, GetConfigurationParams::Response& res, Muse* muse);
		bool setConfigurationParams(SetConfigurationParams::Request& req, SetConfigurationParams::Response& res, Muse* muse);

	};
}

#endif 