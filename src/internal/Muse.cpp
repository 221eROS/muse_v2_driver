#include <muse_v2_driver/Muse.h>

void muse_v2_driver::Muse::setupParams(ros::NodeHandle& node) {

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

bool muse_v2_driver::Muse::stopTransmission(StopTransmission::Request& req, StopTransmission::Response& res, Muse* muse)
{
	ROS_INFO("request: stop_transmission=%s", req.stop_transmission ? "true" : "false");

	if (req.stop_transmission) {
		muse->serial->stopTransmission();
		res.transmission_stopped = true;
	}

	return res.transmission_stopped;
}

bool muse_v2_driver::Muse::shutdown(Shutdown::Request& req, Shutdown::Response& res, Muse* muse, std::vector<ros::Subscriber>& sub_vect)
{
	ROS_INFO("request: shutdown=%s", req.shutdown ? "true" : "false");

	if (req.shutdown) {
		for (auto& s : sub_vect)
			s.shutdown();
		muse->serial->shutdown();
		res.off = true;
		ros::shutdown();
	}

	return res.off;
}

bool muse_v2_driver::Muse::disconnect(Disconnect::Request& req, Disconnect::Response& res, Muse* muse, std::vector<ros::Subscriber>& sub_vect)
{
	ROS_INFO("request: disconnect=%s", req.disconnect ? "true" : "false");

	if (req.disconnect) {
		for (auto& s : sub_vect)
			s.shutdown();

		muse->serial->disconnect();
		res.disconnected = true;
	}
	
	return res.disconnected;
}
