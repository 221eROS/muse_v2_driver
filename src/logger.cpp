#include "ros/ros.h"

#include <muse_v2_driver/Memory.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "logger");

    ros::NodeHandle n;

    muse_v2_driver::Memory mem;

    mem.setupInputCommands(n);

    ros::ServiceClient client = n.serviceClient<muse_v2_driver::MemoryManagement>("logger");

    if (mem.input_command == mem.default_command_list) {
        ROS_ERROR("No request found.");
        ros::shutdown();
        return -1;
    }

    muse_v2_driver::MemoryManagement srv;

    if (mem.input_command.get_available_memory)
        srv.request.get_available_memory = true;

    if (mem.input_command.erase_memory)
        srv.request.erase_memory = true;

    if (mem.input_command.get_files)
        srv.request.get_files = true;

    if (mem.input_command.read_file != 0)
        srv.request.read_file = mem.input_command.read_file;

    if (mem.input_command.read_memory) {
        srv.request.get_files = true;
        srv.request.read_memory = true;
    }
        

    if (client.call(srv))
    {
        if (srv.request.get_available_memory)
            ROS_INFO("Available memory (KB): %.2f", (float)(srv.response.memory / 1024));    

        if (srv.request.erase_memory)
            ROS_INFO("Memory erased: %s", srv.response.memory_erased ? "true" : "false");

        if (srv.request.get_files) {
            ROS_INFO("List of available files:");
            for (int i = 0; i < srv.response.file_list.size(); i++)
            {
                ROS_INFO_STREAM("(" << i+1 <<", " << boost::posix_time::from_iso_string(srv.response.file_list.at(i)) << ")");
            }
        }  

        if (srv.request.read_file != 0) {
            if (srv.response.file_stored)
                ROS_INFO_STREAM("File " << std::to_string(srv.request.read_file) << " stored at " << srv.response.dest_path);
            else
                ROS_INFO("File not stored. Please check the list of available files.");
        }

        if (srv.request.read_memory) {
            if (srv.response.number_of_files_stored > 0)
                ROS_INFO("# of files stored: %d", srv.response.number_of_files_stored);
            else
                ROS_INFO("Not able to store any file.");

            if (srv.response.corrupt_stored_list.size() != 0) {
                    ROS_INFO("Among them, corrupted files follow:");
                    for (int i = 0; i < srv.response.corrupt_stored_list.size(); i++)
                    {
                        ROS_INFO_STREAM("(" << srv.response.corrupt_stored_list.at(i)+1 << ", " << boost::posix_time::from_iso_string(srv.response.file_list.at(srv.response.corrupt_stored_list.at(i))) << ")");
                    }
            }
        }

    }
    else
    {
        ROS_ERROR("Failed to call Logger service.");
        return 1;
    }

    return 0;
}