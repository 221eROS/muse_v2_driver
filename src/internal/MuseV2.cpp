#include <muse_v2_driver/MuseV2.h>

void muse_v2_driver::MuseV2::setupParams(ros::NodeHandle& node) {

	if (!node.getParam("frame_id", params.frame_id))
		ROS_WARN("No frame_id parameter found on server. Use default one.");

	if (!node.getParam("port_name", params.port_name))
		ROS_WARN("No port_name parameter found on server. Use default one.");

	int temp_baudrate;
	if (!node.getParam("baudrate", temp_baudrate))
		ROS_WARN("No baudrate parameter found on server. Use default one.");
	else
		params.baudrate = (uint32_t)temp_baudrate;

	int temp_timeout;
	if (!node.getParam("timeout", temp_timeout))
		ROS_WARN("No timeout parameter found on server. Use default one.");
	else
		params.timeout = (Timeout)(temp_timeout);
}

bool muse_v2_driver::MuseV2::stopTransmission(StopTransmission::Request& req, StopTransmission::Response& res, MuseV2* muse_v2)
{
	ROS_INFO("request: stop_transmission=%s", req.stop_transmission ? "true" : "false");

	if (req.stop_transmission) {
		muse_v2->serial->stopTransmission();
		res.transmission_stopped = true;
	}

	return res.transmission_stopped;
}

bool muse_v2_driver::MuseV2::shutdown(Shutdown::Request& req, Shutdown::Response& res, MuseV2* muse_v2, std::vector<ros::Subscriber>& sub_vect)
{
	ROS_INFO("request: shutdown=%s", req.shutdown ? "true" : "false");

	if (req.shutdown) {
		for (auto& s : sub_vect)
			s.shutdown();
		muse_v2->serial->shutdown();
		res.off = true;
		ros::shutdown();
	}

	return res.off;
}

bool muse_v2_driver::MuseV2::disconnect(Disconnect::Request& req, Disconnect::Response& res, MuseV2* muse_v2, std::vector<ros::Subscriber>& sub_vect)
{
	ROS_INFO("request: disconnect=%s", req.disconnect ? "true" : "false");

	if (req.disconnect) {
		for (auto& s : sub_vect)
			s.shutdown();

		muse_v2->serial->disconnect();
		res.disconnected = true;
	}
	
	return res.disconnected;
}
