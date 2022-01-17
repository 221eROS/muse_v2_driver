#include "ros/ros.h"

#include <muse_v2_driver/Calibration.h>
#include <muse_v2_driver/GetCalibrationParams.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "get_calibration_params");

    ros::NodeHandle n;

    muse_v2_driver::Calibration calib;

    calib.setupInputCommands(n);

    ros::ServiceClient client = n.serviceClient<muse_v2_driver::GetCalibrationParams>("get_calibration_params");

    if (calib.input_command == calib.default_command_list) {
        ROS_ERROR("No request found.");
        ros::shutdown();
        return -1;
    }

    muse_v2_driver::GetCalibrationParams srv;

    if (calib.input_command.get_gyroscope_offset)
        srv.request.get_gyroscope_offset = true;

    if (calib.input_command.get_accelerometer_calib_params)
        srv.request.get_accelerometer_calib_params = true;

    if (calib.input_command.get_magnetometer_calib_params)
        srv.request.get_magnetometer_calib_params = true;


    if (client.call(srv))
    {
        std::string calib_file_path = ros::package::getPath("muse_v2_driver") + "/config/muse_calib.txt";
        std::ofstream muse_calib_file;

        muse_calib_file.open(calib_file_path);

        std::stringstream stream;


        if (srv.request.get_gyroscope_offset) {

            stream.str(std::string());

            stream << "X" << "\t" << "Y" << "\t" << "Z" << "\n";
            stream << std::setprecision(6) << std::fixed << srv.response.current_gyro_offset[0] << "\t" << srv.response.current_gyro_offset[1] << "\t" << srv.response.current_gyro_offset[2];

            ROS_INFO_STREAM("Gyroscope Calibration params: \n" << stream.str());

            if (!muse_calib_file) {
                ROS_ERROR("Error: file could not be opened.");
                return 1;
            }

            muse_calib_file << "Accelerometer Calibration params: \n" << stream.str() << "\n";

        }

        if (srv.request.get_accelerometer_calib_params) {

            stream.str(std::string());

            stream << "X" << "\t" << "Y" << "\t" << "Z" << "\t" << "offset" << "\n";
            stream << std::setprecision(6) << std::fixed << srv.response.current_acc_params[0] << "\t" << srv.response.current_acc_params[1] << "\t" << srv.response.current_acc_params[2] << "\t" << srv.response.current_acc_params[9] << "\n";
            stream << srv.response.current_acc_params[3] << "\t" << srv.response.current_acc_params[4] << "\t" << srv.response.current_acc_params[5] << "\t" << srv.response.current_acc_params[10] << "\n";
            stream << srv.response.current_acc_params[6] << "\t" << srv.response.current_acc_params[7] << "\t" << srv.response.current_acc_params[8] << "\t" << srv.response.current_acc_params[11];

            ROS_INFO_STREAM("Accelerometer Calibration params: \n" << stream.str());

            if (!muse_calib_file) {
                ROS_ERROR("Error: file could not be opened.");
                return 1;
            }

            muse_calib_file << "Accelerometer Calibration params: \n" << stream.str() << "\n";
            
        }

        if (srv.request.get_magnetometer_calib_params) {

            stream.str(std::string());

            stream << "X" << "\t" << "Y" << "\t" << "Z" << "\t" << "offset" << "\n";
            stream << std::setprecision(6) << std::fixed << srv.response.current_mag_params[0] << "\t" << srv.response.current_mag_params[1] << "\t" << srv.response.current_mag_params[2] << "\t" << srv.response.current_mag_params[9] << "\n";
            stream << srv.response.current_mag_params[3] << "\t" << srv.response.current_mag_params[4] << "\t" << srv.response.current_mag_params[5] << "\t" << srv.response.current_mag_params[10] << "\n";
            stream << srv.response.current_mag_params[6] << "\t" << srv.response.current_mag_params[7] << "\t" << srv.response.current_mag_params[8] << "\t" << srv.response.current_mag_params[11];

            ROS_INFO_STREAM("Magnetometer Calibration params: \n" << stream.str());

            if (!muse_calib_file) {
                ROS_ERROR("Error: file could not be opened.");
                return 1;
            }

            muse_calib_file << "Magnetometer Calibration params: \n" << stream.str() << "\n";

        }

        muse_calib_file.close();

    }
    else
    {
        ROS_ERROR("Failed to call Get Calibration Params service.");
        return 1;
    }

    return 0;
}