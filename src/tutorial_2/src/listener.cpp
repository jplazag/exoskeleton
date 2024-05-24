/* Listener

Code file designed to test the comunication between the PC and the circuit board
Receives the data from q_control_publisher topic*/


#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"


void chatterCallback(const std_msgs::String::ConstPtr &msg)
{
    ROS_INFO("I heard: [%s]", msg->data.c_str());
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "listener");

    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("q_control_publisher", 1, chatterCallback);

    ros::spin();

    return 0;
}
