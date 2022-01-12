#include <muse_v2_driver/Configuration.h>

void muse_v2_driver::Configuration::setupInputCommands(ros::NodeHandle& node) {
	node.getParam("get_gyroscope_full_scale", input_command.get_gyroscope_full_scale);
	node.getParam("get_accelerometer_full_scale", input_command.get_accelerometer_full_scale);
	node.getParam("get_accelerometer_hdr_full_scale", input_command.get_accelerometer_hdr_full_scale);
	node.getParam("get_magnetometer_full_scale", input_command.get_magnetometer_full_scale);
	node.getParam("get_log_mode", input_command.get_log_mode);
	node.getParam("get_log_frequency", input_command.get_log_frequency);

	int temp_gyr_full_scale, temp_acc_full_scale, temp_acc_hdr_full_scale, temp_mag_full_scale, temp_log_mode, temp_log_freq;

	if (node.getParam("gyroscope_full_scale", temp_gyr_full_scale))
		input_command.gyroscope_full_scale = temp_gyr_full_scale;
	if (node.getParam("accelerometer_full_scale", temp_acc_full_scale))
		input_command.accelerometer_full_scale = temp_acc_full_scale;
	if (node.getParam("accelerometer_hdr_full_scale_", temp_acc_hdr_full_scale))
		input_command.accelerometer_hdr_full_scale = temp_acc_hdr_full_scale;
	if (node.getParam("magnetometer_full_scale", temp_mag_full_scale))
		input_command.magnetometer_full_scale = temp_mag_full_scale;
	if (node.getParam("log_mode", temp_log_mode))
		input_command.log_mode = temp_log_mode;
	if (node.getParam("log_frequency", temp_log_freq))
		input_command.log_frequency = temp_log_freq;
}

bool muse_v2_driver::Configuration::getConfigurationParams(GetConfigurationParams::Request& req, GetConfigurationParams::Response& res, Muse* muse)
{
	bool out = false;

	ROS_INFO("request: get_gyroscope_full_scale=%s, get_accelerometer_full_scale=%s, get_accelerometer_hdr_full_scale=%s, get_magnetometer_full_scale=%s, get_log_mode=%s, get_log_frequency=%s",
		req.get_gyroscope_full_scale ? "true" : "false",
		req.get_accelerometer_full_scale ? "true" : "false",
		req.get_accelerometer_hdr_full_scale ? "true" : "false",
		req.get_magnetometer_full_scale ? "true" : "false",
		req.get_log_mode ? "true" : "false",
		req.get_log_frequency ? "true" : "false"
	);

	if (req.get_gyroscope_full_scale) {
		if (!received_command.get_gyroscope_full_scale) {
			received_command.get_gyroscope_full_scale = true;
			ROS_INFO("Asking Gyroscope full-scale.");
		}
		if (muse->serial->getGyroscopeFullScale() > 0) {
			res.gyroscope_full_scale = muse->serial->getGyroscopeFullScale();
			out = true;
			ROS_INFO("Sending back Gyroscope Full-Scale [dps]: %u", res.gyroscope_full_scale);
		}
	}
	if (req.get_accelerometer_full_scale) {
		if (!received_command.get_accelerometer_full_scale) {
			received_command.get_accelerometer_full_scale = true;
			ROS_INFO("Asking Accelerometer full-scale.");
		}
		if (muse->serial->getAccFullScale()>0) {
			res.accelerometer_full_scale = muse->serial->getAccFullScale();
			out = true;
			ROS_INFO("Sending back Accelerometer Full-Scale [g]: %u", res.accelerometer_full_scale);
		}
	}
	if (req.get_accelerometer_hdr_full_scale) {
		if (!received_command.get_accelerometer_hdr_full_scale) {
			received_command.get_accelerometer_hdr_full_scale = true;
			ROS_INFO("Asking Accelerometer HDR full-scale.");
		}
		if (muse->serial->getAccHdrFullScale()>0) {
			res.accelerometer_hdr_full_scale = muse->serial->getAccHdrFullScale();
			out = true;
			ROS_INFO("Sending back Accelerometer HDR Full-Scale [g]: %u", res.accelerometer_hdr_full_scale);
		}
	}
	if (req.get_magnetometer_full_scale) {
		if (!received_command.get_magnetometer_full_scale) {
			received_command.get_magnetometer_full_scale = true;
			ROS_INFO("Asking Magnetometer full-scale.");
		}
		if (muse->serial->getMagnetometerFullScale()>0) {
			res.magnetometer_full_scale = muse->serial->getMagnetometerFullScale();
			out = true;
			ROS_INFO("Sending back Magnetometer Full-Scale [G]: %u", res.magnetometer_full_scale);
		}
	}
	if (req.get_log_mode) {
		if (!received_command.get_log_mode) {
			received_command.get_log_mode = true;
			ROS_INFO("Asking Log Mode.");
		}
		if (muse->serial->getLogMode() < UINT8_MAX) {
			res.log_mode = muse->serial->getLogMode();
			out = true;
			ROS_INFO("Sending back Log Mode: %u", res.log_mode);
		}
	}
	if (req.get_log_frequency) {
		if (!received_command.get_log_frequency) {
			received_command.get_log_frequency = true;
			ROS_INFO("Asking Log Frequency.");
		}
		if (muse->serial->getLogFrequency() < UINT8_MAX) {
			res.log_frequency = muse->serial->getLogFrequency();
			out = true;
			ROS_INFO("Sending back Log Frequency: %u", res.log_frequency);
		}
	}
	return out;
}

bool muse_v2_driver::Configuration::setConfigurationParams(SetConfigurationParams::Request& req, SetConfigurationParams::Response& res, Muse* muse)
{
	bool out = false;

	ROS_INFO("request: set_gyroscope_full_scale=%s, set_accelerometer_full_scale=%s, set_accelerometer_hdr_full_scale=%s, set_magnetometer_full_scale=%s, set_log_mode=%s, set_log_frequency=%s",
		req.set_gyroscope_full_scale ? "true" : "false",
		req.set_accelerometer_full_scale ? "true" : "false",
		req.set_accelerometer_hdr_full_scale ? "true" : "false",
		req.set_magnetometer_full_scale ? "true" : "false",
		req.set_log_mode ? "true" : "false",
		req.set_log_frequency ? "true" : "false"
	);

	if (req.set_gyroscope_full_scale) {
		if (received_command.gyroscope_full_scale == 0) {
			received_command.gyroscope_full_scale = req.gyroscope_full_scale;
			ROS_INFO("Setting Gyroscope Full-Scale.");
		}
		if (muse->serial->setGyroscopeFullScale(req.gyroscope_full_scale)) {
			out = true;
			res.gyroscope_full_scale_changed = true;
			ROS_INFO("Gyroscope Full-Scale value changed: %s", res.gyroscope_full_scale_changed ? "true" : "false");
		}
	}
	if (req.set_accelerometer_full_scale) {
		if (received_command.accelerometer_full_scale == 0) {
			received_command.accelerometer_full_scale = req.accelerometer_full_scale;
			ROS_INFO("Setting Accelerometer Full-Scale.");
		}
		if (muse->serial->setAccelerometerFullScale(req.accelerometer_full_scale)) {
			out = true;
			res.accelerometer_full_scale_changed = true;
			ROS_INFO("Accelerometer Full-Scale value changed: %s", res.accelerometer_full_scale_changed ? "true" : "false");
		}
	}
	if (req.set_accelerometer_hdr_full_scale) {
		if (received_command.accelerometer_hdr_full_scale == 0) {
			received_command.accelerometer_hdr_full_scale = req.accelerometer_hdr_full_scale;
			ROS_INFO("Setting Accelerometer Full-Scale.");
		}
		if (muse->serial->setAccelerometerHDRFullScale(req.accelerometer_hdr_full_scale)) {
			out = true;
			res.accelerometer_hdr_full_scale_changed = true;
			ROS_INFO("Accelerometer HDR Full-Scale value changed: %s", res.accelerometer_hdr_full_scale_changed ? "true" : "false");
		}
	}
	if (req.set_magnetometer_full_scale) {
		if (received_command.magnetometer_full_scale == 0) {
			received_command.magnetometer_full_scale = req.magnetometer_full_scale;
				ROS_INFO("Setting Magnetometer Full-Scale.");
		}
		if (muse->serial->setMagnetometerFullScale(req.magnetometer_full_scale)) {
			out = true;
			res.magnetometer_full_scale_changed = true;
			ROS_INFO("Magnetometer Full-Scale value changed: %s", res.magnetometer_full_scale_changed ? "true" : "false");
		}
	}
	if (req.set_log_mode) {
		if (received_command.log_mode == UINT8_MAX) {
			received_command.log_mode = req.log_mode;
			ROS_INFO("Setting Log Mode.");
		}
		if (muse->serial->setLogMode(req.log_mode)) {
			out = true;
			res.log_mode_changed = true;
			ROS_INFO("Log Mode changed: %s", res.log_mode_changed ? "true" : "false");
		}
	}
	if (req.set_log_frequency) {
		if (received_command.log_frequency == UINT8_MAX) {
			received_command.log_frequency = req.log_frequency;
			ROS_INFO("Setting Log Frequency.");
		}
		if (muse->serial->setLogFrequency(req.log_frequency)) {
			out = true;
			res.log_frequency_changed = true;
			ROS_INFO("Log Frequency changed: %s", res.log_frequency_changed ? "true" : "false");
		}
	}
	return out;
}
		