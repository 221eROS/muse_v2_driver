#include <muse_v2_driver/Calibration.h>

void muse_v2_driver::Calibration::setupInputCommands(ros::NodeHandle& node) {

	node.getParam("get_gyroscope_offset", input_command.get_gyroscope_offset);
	node.getParam("get_accelerometer_calib_params", input_command.get_accelerometer_calib_params);
	node.getParam("get_magnetometer_calib_params", input_command.get_magnetometer_calib_params);

}

bool muse_v2_driver::Calibration::getCalibrationParams(GetCalibrationParams::Request& req, GetCalibrationParams::Response& res, Muse* muse) {
	bool out = false;

	ROS_INFO("request: get_gyroscope_offset=%s, get_accelerometer_calib_params=%s, get_magnetometer_calib_params=%s",
		req.get_gyroscope_offset ? "true" : "false",
		req.get_accelerometer_calib_params ? "true" : "false",
		req.get_magnetometer_calib_params ? "true" : "false"
	);

	if (req.get_gyroscope_offset) {
		if(!received_command.get_gyroscope_offset) {
			received_command.get_gyroscope_offset = true;
			ROS_INFO("Asking Gyroscope Offset.");
		}
		if (!muse->serial->getGyroscopeOffset().empty()) {
			for (const auto& value : muse->serial->getGyroscopeOffset())
				res.current_gyro_offset.push_back(value);
			out = true;
			ROS_INFO("Sending back Gyroscope Calib Params.");
		}
	}

	if (req.get_accelerometer_calib_params) {
		if (!received_command.get_accelerometer_calib_params) {
			received_command.get_accelerometer_calib_params = true;
			ROS_INFO("Asking Acceleromter Calibration Params.");
		}
		if (!muse->serial->getAccelerometerCalibParams().empty()) {
			for (auto& value : muse->serial->getAccelerometerCalibParams())
				res.current_acc_params.push_back(value);
			out = true;
			ROS_INFO("Sending back Accelerometer Calib Params.");
		}
	}

	if (req.get_magnetometer_calib_params) {
		if (!received_command.get_magnetometer_calib_params) {
			received_command.get_magnetometer_calib_params = true;
			ROS_INFO("Asking Magnetometer Calibration Params.");
		}
		if (!muse->serial->getMagnetometerCalibParams().empty()) {
			for (auto& value : muse->serial->getMagnetometerCalibParams())
				res.current_mag_params.push_back(value);
			out = true;
			ROS_INFO("Sending back Magnetometer Calib Params.");
		}
	}

	return out;
}
