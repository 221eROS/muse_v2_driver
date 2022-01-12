#ifndef MUSE_H
#define MUSE_H

#include <ros/ros.h>

#include <muse/Muse_SerialConnection.h>

#include <muse_v2_driver/StopTransmission.h>
#include <muse_v2_driver/Shutdown.h>
#include <muse_v2_driver/Disconnect.h>

using namespace Muse;
using namespace std;

const string DEFAULT_FRAME_ID = "";
const string DEFAULT_PORT_NAME = "";
const uint32_t DEFAULT_BAUDRATE = 115200;

namespace muse_v2_driver {

	class Muse
	{
	public:

		struct Params
		{
			string frame_id = DEFAULT_FRAME_ID;
			string port_name = DEFAULT_PORT_NAME;
			uint32_t baudrate = DEFAULT_BAUDRATE;
			Timeout timeout = Timeout();
		};

		Muse(ros::NodeHandle& node) {
			setupParams(node);
			serial = new Muse_SerialConnection(params.port_name, params.baudrate, params.timeout);
		};

		~Muse() = default;

		Params params;
		Muse_SerialConnection *serial;

		void setupParams(ros::NodeHandle& node);
		bool stopTransmission(StopTransmission::Request& req, StopTransmission::Response& res, Muse* muse);
		bool shutdown(Shutdown::Request& req, Shutdown::Response& res, Muse* muse, std::vector<ros::Subscriber>& sub_vect);
		bool disconnect(Disconnect::Request& req, Disconnect::Response& res, Muse* muse, std::vector<ros::Subscriber>& sub_vect);

	};
}

#endif 