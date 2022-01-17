#include <muse_v2_driver/Stream.h>
#include <muse_v2_driver/MuseV2.h>
#include <muse_v2_driver/Miscellaneous.h>
#include <muse_v2_driver/Configuration.h>
#include <muse_v2_driver/Calibration.h>
#include <muse_v2_driver/Memory.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "muse");

	ros::NodeHandle n;

	muse_v2_driver::MuseV2 muse_v2(n);

	ROS_INFO("Port name: %s", muse_v2.params.port_name);
	ROS_INFO("Baudrate: %d", muse_v2.params.baudrate);

	int publisher_queue_size;

	if (!n.getParam("publisher_queue_size", publisher_queue_size))
		ROS_WARN("No publisher_queue_size parameter found on server. Use default one.");

	muse_v2_driver::Stream stream(n, publisher_queue_size);
	muse_v2_driver::Miscellaneous misc;
	muse_v2_driver::Configuration config;
	muse_v2_driver::Calibration calib;
	muse_v2_driver::Memory mem;

	std::vector<ros::Subscriber> sub_vect;

	ros::Subscriber stream_sub = n.subscribe<muse_v2_driver::Transmission>("start_transmission", 1, boost::bind(&muse_v2_driver::Stream::StreamRawData, &stream, _1, &muse_v2));
	sub_vect.push_back(stream_sub);

	ros::ServiceServer stop_transmission_srv = n.advertiseService<muse_v2_driver::StopTransmission::Request, muse_v2_driver::StopTransmission::Response>
		("stop_transmission", boost::bind(&muse_v2_driver::MuseV2::stopTransmission, &muse_v2, _1, _2, &muse_v2));

	ros::ServiceServer shutdown_srv = n.advertiseService<muse_v2_driver::Shutdown::Request, muse_v2_driver::Shutdown::Response>
		("shutdown", boost::bind(&muse_v2_driver::MuseV2::shutdown, &muse_v2, _1, _2, &muse_v2, sub_vect));

	ros::ServiceServer battery_srv = n.advertiseService<muse_v2_driver::Battery::Request, muse_v2_driver::Battery::Response>
		("battery", boost::bind(&muse_v2_driver::Miscellaneous::getBattery, &misc, _1, _2, &muse_v2));

	ros::ServiceServer get_config_params_srv = n.advertiseService<muse_v2_driver::GetConfigurationParams::Request, muse_v2_driver::GetConfigurationParams::Response>
		("get_configuration_params", boost::bind(&muse_v2_driver::Configuration::getConfigurationParams, &config, _1, _2, &muse_v2));

	ros::ServiceServer set_config_params_srv = n.advertiseService<muse_v2_driver::SetConfigurationParams::Request, muse_v2_driver::SetConfigurationParams::Response>
		("set_configuration_params", boost::bind(&muse_v2_driver::Configuration::setConfigurationParams, &config, _1, _2, &muse_v2));

	ros::ServiceServer get_calib_params_srv = n.advertiseService<muse_v2_driver::GetCalibrationParams::Request, muse_v2_driver::GetCalibrationParams::Response>
		("get_calibration_params", boost::bind(&muse_v2_driver::Calibration::getCalibrationParams, &calib, _1, _2, &muse_v2));

	ros::ServiceServer logger_srv = n.advertiseService<muse_v2_driver::MemoryManagement::Request, muse_v2_driver::MemoryManagement::Response>
		("logger", boost::bind(&muse_v2_driver::Memory::logger, &mem, _1, _2, &muse_v2));


	ROS_INFO("Muse V2 ready.");

	ros::spin();

	return 0;

}
