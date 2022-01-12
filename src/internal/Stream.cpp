#include <muse_v2_driver/Stream.h>

muse_v2_driver::Stream::Stream(ros::NodeHandle& node, int pub_queue_size){

	pub_acceleration = node.advertise<geometry_msgs::Vector3Stamped>("imu/acceleration", pub_queue_size);;
	pub_angular_velocity = node.advertise<geometry_msgs::Vector3Stamped>("imu/angular_velocity", pub_queue_size);;
	pub_imu = node.advertise<sensor_msgs::Imu>("imu/data", pub_queue_size);
	pub_mag = node.advertise<sensor_msgs::MagneticField>("imu/mag", pub_queue_size);
	pub_quaternion = node.advertise<geometry_msgs::QuaternionStamped>("imu/quaternion", pub_queue_size);
	pub_rpy = node.advertise<geometry_msgs::Vector3Stamped>("imu/rpy", pub_queue_size);

}

void muse_v2_driver::Stream::setupInputCommands(ros::NodeHandle& node) {

	node.getParam("get_imu", input_command.get_imu);
	node.getParam("get_quaternion", input_command.get_quaternion);
	node.getParam("get_angular_velocity", input_command.get_angular_velocity);
	node.getParam("get_acceleration", input_command.get_acceleration);
	node.getParam("get_mag", input_command.get_mag);
	node.getParam("get_rpy", input_command.get_rpy);
}

bool muse_v2_driver::Stream::isFrequencyAdmissible(int frequency) {
	bool result = false;

	if (frequency <= Muse_HW::MAX_STREAM_FREQUENCY)
		result = true;
	else { ROS_ERROR("Invalid frequency."); }

	return result;

}

void muse_v2_driver::Stream::StreamRawData(const Transmission::ConstPtr& msg, Muse* muse) {

	if (!msg->transmission.data.empty() && std::find(msg->transmission.data.begin(), msg->transmission.data.end(), static_cast<int>(CommandType::GET_IMU)) != msg->transmission.data.end()) {

		if (!received_command.get_imu) {
			received_command.get_imu = true;
			ROS_INFO("Start Imu transmission.");
		}

		Imu imu = muse->serial->getIMU(msg->frequency);

		imu_msg.header.stamp = ros::Time::now();
		imu_msg.header.frame_id = muse->params.frame_id;

		// Quaternion
		imu_msg.orientation.w = imu.quaternion.w;
		imu_msg.orientation.x = imu.quaternion.x;
		imu_msg.orientation.y = imu.quaternion.y;
		imu_msg.orientation.z = imu.quaternion.z;

		// Angular velocity
		imu_msg.angular_velocity.x = imu.gyroscope.x;
		imu_msg.angular_velocity.y = imu.gyroscope.y;
		imu_msg.angular_velocity.z = imu.gyroscope.z;

		// Acceleration
		imu_msg.linear_acceleration.x = imu.linear_acceleration.x;
		imu_msg.linear_acceleration.y = imu.linear_acceleration.y;
		imu_msg.linear_acceleration.z = imu.linear_acceleration.z;

		pub_imu.publish(imu_msg);
	}

	if (!msg->transmission.data.empty() && std::find(msg->transmission.data.begin(), msg->transmission.data.end(), static_cast<int>(CommandType::GET_QUATERNION)) != msg->transmission.data.end()) {
		if (!received_command.get_quaternion) {
			received_command.get_quaternion = true;
			ROS_INFO("Start Quaternion transmission.");
		}

		Quaternion quaternion = muse->serial->getQuaternion(msg->frequency);

		quaternion_msg.header.stamp = ros::Time::now();
		quaternion_msg.header.frame_id = muse->params.frame_id;

		// Quaternion
		quaternion_msg.quaternion.w = quaternion.w;
		quaternion_msg.quaternion.x = quaternion.x;
		quaternion_msg.quaternion.y = quaternion.y;
		quaternion_msg.quaternion.z = quaternion.z;

		pub_quaternion.publish(quaternion_msg);

	}	

	if (!msg->transmission.data.empty() && std::find(msg->transmission.data.begin(), msg->transmission.data.end(), static_cast<int>(CommandType::GET_ANGULAR_VELOCITY)) != msg->transmission.data.end()) {
		if (!received_command.get_angular_velocity) {
			received_command.get_angular_velocity = true;
			ROS_INFO("Start Angular Velocity transmission.");
		}

		AngularVelocity angular_velocity = muse->serial->getAngularVelocity(msg->frequency);

		angular_velocity_msg.header.stamp = ros::Time::now();
		angular_velocity_msg.header.frame_id = muse->params.frame_id;

		// Angular velocity
		angular_velocity_msg.vector.x = angular_velocity.x;
		angular_velocity_msg.vector.y = angular_velocity.y;
		angular_velocity_msg.vector.z = angular_velocity.z;

		pub_angular_velocity.publish(angular_velocity_msg);
	}
		
	if (!msg->transmission.data.empty() && std::find(msg->transmission.data.begin(), msg->transmission.data.end(), static_cast<int>(CommandType::GET_ACCELERATION)) != msg->transmission.data.end()) {
		if (!received_command.get_acceleration) {
			received_command.get_acceleration = true;
			ROS_INFO("Start Acceleration transmission.");
		}

		Acceleration acceleration = muse->serial->getAcceleration(msg->frequency);

		acceleration_msg.header.stamp = ros::Time::now();
		acceleration_msg.header.frame_id = muse->params.frame_id;

		// Acceleration
		acceleration_msg.vector.x = acceleration.x;
		acceleration_msg.vector.y = acceleration.y;
		acceleration_msg.vector.z = acceleration.z;

		pub_acceleration.publish(acceleration_msg);
	}
		
	if (!msg->transmission.data.empty() && std::find(msg->transmission.data.begin(), msg->transmission.data.end(), static_cast<int>(CommandType::GET_MAG)) != msg->transmission.data.end()) {
		if (!received_command.get_mag) {
			received_command.get_mag = true;
			ROS_INFO("Start Mag transmission.");
		}

		MagneticField mag = muse->serial->getMag(msg->frequency);

		mag_msg.header.stamp = ros::Time::now();
		mag_msg.header.frame_id = muse->params.frame_id;

		// Magnetic Field
		mag_msg.magnetic_field.x = mag.x;
		mag_msg.magnetic_field.y = mag.y;
		mag_msg.magnetic_field.z = mag.z;

		pub_mag.publish(mag_msg);

	}
		
	if (!msg->transmission.data.empty() && std::find(msg->transmission.data.begin(), msg->transmission.data.end(), static_cast<int>(CommandType::GET_RPY)) != msg->transmission.data.end()) {
		if (!received_command.get_rpy) {
			received_command.get_rpy = true;
			ROS_INFO("Start RPY transmission.");
		}

		EulerAngles rpy = muse->serial->getRPY(msg->frequency);

		rpy_msg.header.stamp = ros::Time::now();
		rpy_msg.header.frame_id = muse->params.frame_id;

		// RPY
		rpy_msg.vector.x = rpy.roll;
		rpy_msg.vector.y = rpy.pitch;
		rpy_msg.vector.z = rpy.yaw;

		pub_rpy.publish(rpy_msg);
	}	
}