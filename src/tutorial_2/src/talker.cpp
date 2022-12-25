#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"

#include <sstream>
#include <iostream>

using namespace std; 

int main (int argc, char **argv)
{
    ros::init(argc, argv, "talker");

    ros::NodeHandle n; // Initializes the node

    ros::Publisher q_control_pub = n.advertise<std_msgs::Float64>("q_control_publisher", 1000); 
    //Tell the master that we are going to be publishing a message of type std_msgs/Float64 on the topic /q_control_publisher
    // The second argument is the size of our publishing queue. 
    //In this case if we are publishing too quickly it will buffer up a maximum of 1000 messages before beginning to throw away old ones. 


    // NodeHandle::advertise() returns a ros::Publisher object
    //t contains a publish() method that lets you publish messages onto the topic it was created with
    //when it goes out of scope, it will automatically unadvertise



    ros::Rate loop_rate(10); //specify a frequency that you would like to loop at
    
    int count = 0;
    double f;
    while (ros::ok()) //Ctrl-C handling which will cause ros::ok() to return false if that happens. 
    {
        std_msgs::Float64 msg;

        // std::stringstream ss;
        // ss << 6 << count;
        cout << "Insert the data";
        cin >> f;
        msg.data = f; //We broadcast a message on ROS using a message-adapted class

        // ROS_INFO("%s", msg.data.c_str());

        q_control_pub.publish(msg);

        ros::spinOnce(); //if you were to add a subscription into this application, and did not have ros::spinOnce() here, 
        //your callbacks would never get called. So, add it for good measure. 

        loop_rate.sleep(); //Now we use the ros::Rate object to sleep for the time remaining to let us hit our 10Hz publish rate.
        ++count;
    }

    return 0;
}