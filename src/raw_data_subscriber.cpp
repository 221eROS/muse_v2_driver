#include <ros/ros.h>

#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>

/**
 * Subscriber callbacks
 */

void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
    ROS_INFO("Linear acc: %.3f,%.3f,%.3f [m/s^2] - Ang. vel: %.3f,%.3f,%.3f [deg/sec] - Orient. Quat: %.3f,%.3f,%.3f,%.3f",
        msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z,
        msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z,
        msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
}

void accCallback(const geometry_msgs::Vector3StampedConstPtr& msg) {
    ROS_INFO("Linear acc: %.3f,%.3f,%.3f [m/s^2]",
        msg->vector.x, msg->vector.y, msg->vector.z);
}

void angVelCallback(const geometry_msgs::Vector3StampedConstPtr& msg) {
    ROS_INFO("Ang. vel: %.3f,%.3f,%.3f [deg/sec]",
        msg->vector.x, msg->vector.y, msg->vector.z);
}

void quatCallback(const geometry_msgs::QuaternionStampedConstPtr& msg) {
    ROS_INFO("Orient. Quat: %.3f,%.3f,%.3f,%.3f",
        msg->quaternion.x, msg->quaternion.y, msg->quaternion.z, msg->quaternion.w);
}

void rpyCallback(const geometry_msgs::Vector3StampedConstPtr& msg) {
    ROS_INFO("RPY: %.3f,%.3f,%.3f",
        msg->vector.x, msg->vector.y, msg->vector.z);
}

void magCallback(const sensor_msgs::MagneticField::ConstPtr& msg) {
    ROS_INFO("Mag. Field: %.3f,%.3f,%.3f [uT]",
        msg->magnetic_field.x * 1e-6, msg->magnetic_field.y * 1e-6, msg->magnetic_field.z * 1e-6);
}

/**
 * Node main function
 */

int main(int argc, char** argv) {
    ros::init(argc, argv, "muse_raw_data_subscriber");
    ros::NodeHandle n;

    ros::Subscriber subImu = n.subscribe("/imu/data", 10, imuCallback);
    ros::Subscriber subAcc = n.subscribe("/imu/linear_acceleration", 10, accCallback);
    ros::Subscriber subAngVel = n.subscribe("/imu/angular_velocity", 10, angVelCallback);
    ros::Subscriber subQuaternion = n.subscribe("/imu/quaternion", 10, quatCallback);
    ros::Subscriber subRPY = n.subscribe("/imu/rpy", 10, rpyCallback);
    ros::Subscriber subMag = n.subscribe("/zed/zed_node/imu/mag", 10, magCallback);

    ros::spin();

    return 0;
}