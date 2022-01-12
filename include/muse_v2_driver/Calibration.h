#ifndef CALIBRATION_H
#define CALIBRATION_H

#include <ros/package.h>
#include <muse_v2_driver/Muse.h>
#include <muse_v2_driver/GetCalibrationParams.h>
#include <fstream>

namespace muse_v2_driver {

	class Calibration
	{
	public:

		struct CommandList
		{
			bool get_gyroscope_offset = false;
			bool get_accelerometer_calib_params = false;
			bool get_magnetometer_calib_params = false;

			bool calibrate_gyroscope = false;
			bool calibrate_acceleromter = false;
			bool calibrate_magnetometer = false;

			bool operator==(const CommandList& rhs) const {
				return  (
					(get_gyroscope_offset == rhs.get_gyroscope_offset) &&
					(get_accelerometer_calib_params == rhs.get_accelerometer_calib_params) &&
					(get_magnetometer_calib_params == rhs.get_magnetometer_calib_params) &&
					(calibrate_gyroscope == rhs.calibrate_gyroscope) &&
					(calibrate_acceleromter == rhs.calibrate_acceleromter) &&
					(calibrate_magnetometer == rhs.calibrate_magnetometer)
					);
			}

		} default_command_list;

		CommandList input_command, received_command;

		Calibration() = default;
		~Calibration() = default;

		void setupInputCommands(ros::NodeHandle& node);
		bool getCalibrationParams(GetCalibrationParams::Request& req, GetCalibrationParams::Response& res, Muse* muse);
	};
}

#endif 