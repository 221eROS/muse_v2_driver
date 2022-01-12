#include "ros/ros.h"
#include "muse_v2_driver/Shutdown.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "shutdown");

    ros::NodeHandle n;

    ros::ServiceClient client = n.serviceClient<muse_v2_driver::Shutdown>("shutdown");

    muse_v2_driver::Shutdown srv;

    srv.request.shutdown = true;

    client.call(srv);
    ROS_INFO("Shutdown: true");

    return 0;
}