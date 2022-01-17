#ifndef MUSE_ROS_V2_H
#define MUSE_ROS_V2_H

#include <ros/ros.h>

#include <muse_v2/MuseV2_SerialConnection.h>

#include <muse_v2_driver/StopTransmission.h>
#include <muse_v2_driver/Shutdown.h>
#include <muse_v2_driver/Disconnect.h>

using namespace MuseV2;
using namespace std;

const string DEFAULT_FRAME_ID = "";
const string DEFAULT_PORT_NAME = "";
const uint32_t DEFAULT_BAUDRATE = 115200;

namespace muse_v2_driver {

	class MuseV2
	{
	public:

		struct Params
		{
			string frame_id = DEFAULT_FRAME_ID;
			string port_name = DEFAULT_PORT_NAME;
			uint32_t baudrate = DEFAULT_BAUDRATE;
			Timeout timeout = Timeout();
		};

		MuseV2(ros::NodeHandle& node) {
			setupParams(node);
			serial = new MuseV2_SerialConnection(params.port_name, params.baudrate, params.timeout);
		};

		~MuseV2() = default;

		Params params;
		MuseV2_SerialConnection *serial;

		void setupParams(ros::NodeHandle& node);
		bool stopTransmission(StopTransmission::Request& req, StopTransmission::Response& res, MuseV2* muse_v2);
		bool shutdown(Shutdown::Request& req, Shutdown::Response& res, MuseV2* muse_v2, std::vector<ros::Subscriber>& sub_vect);
		bool disconnect(Disconnect::Request& req, Disconnect::Response& res, MuseV2* muse_v2, std::vector<ros::Subscriber>& sub_vect);

	};
}

#endif 