#ifndef STREAM_H
#define STREAM_H

#include <muse_v2_driver/MuseV2.h>

#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <muse_v2_driver/Transmission.h>

#include <muse_v2/MuseV2_HW.h>

#include <tf/tf.h>

namespace muse_v2_driver {

	class Stream
	{
	public:

		struct CommandList
		{
			bool get_imu = false;
			bool get_quaternion = false;
			bool get_angular_velocity = false;
			bool get_acceleration = false;
			bool get_mag = false;
			bool get_rpy = false;

			bool operator==(const CommandList& rhs) const {
				return  (
					(get_imu == rhs.get_imu) &&
					(get_quaternion == rhs.get_quaternion) &&
					(get_angular_velocity == rhs.get_angular_velocity) &&
					(get_angular_velocity == rhs.get_angular_velocity) &&
					(get_acceleration == rhs.get_acceleration) &&
					(get_mag == rhs.get_mag) &&
					(get_rpy == rhs.get_rpy)
					);
			}

		} default_command_list;

		enum CommandType {
			GET_IMU,
			GET_QUATERNION,
			GET_ANGULAR_VELOCITY,
			GET_ACCELERATION,
			GET_MAG,
			GET_RPY
		};

		CommandList input_command, received_command;

		geometry_msgs::QuaternionStamped quaternion_msg;
		geometry_msgs::Vector3Stamped rpy_msg;
		geometry_msgs::Vector3Stamped angular_velocity_msg;
		geometry_msgs::Vector3Stamped acceleration_msg;
		sensor_msgs::MagneticField mag_msg;
		sensor_msgs::Imu imu_msg;

		ros::Publisher pub_acceleration, pub_angular_velocity, pub_imu, pub_mag, pub_quaternion, pub_rpy;

		Stream() = default;
		Stream(ros::NodeHandle& node, int pub_queue_size = 10);
		~Stream() = default;

		void setupInputCommands(ros::NodeHandle& node);
		bool isFrequencyAdmissible(int frequency);
		void StreamRawData(const Transmission::ConstPtr& msg, MuseV2* muse_v2);

	};
}

#endif 