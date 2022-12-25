#include "ros/ros.h"
#include "tutorial_2/SetBool.h"

bool attach_deattach(tutorial_2::SetBool::Request  &req,
        tutorial_2::SetBool::Response &res)
    {   
        res.success = req.data;
        ROS_INFO("request = %d", req.data);
        return true;
    }

    int main(int argc, char **argv)
    {
        ros::init(argc, argv, "server");
        ros::NodeHandle n;
         
        ros::ServiceServer service = n.advertiseService("service_server", attach_deattach);
        ROS_INFO("Ready to work.");
        ros::spin();

        return 0;
    }