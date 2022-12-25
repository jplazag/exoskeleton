#include "ros/ros.h"
#include "tutorial_2/SetBool.h"
#include <cstdlib>

int main(int argc, char **argv)

{
    ros::init(argc, argv, "service_client");
    if (argc != 2)
    {
        ROS_INFO("usage: enter a 1 if you want to attach the servo or enter a 0 if you want to deattach");
        return 1;
    }
    ros::NodeHandle n;

    ros::ServiceClient client = n.serviceClient<tutorial_2::SetBool>("service_server");

    tutorial_2::SetBool srv;
    srv.request.data = (bool)atoll(argv[1]);
    
    if (client.call(srv))
    {
        ROS_INFO("Succes to call service set_bool");
    }
    else
    {
        ROS_ERROR("Failed to call service set_bool");
        return 1;
    }
    return 0;
}