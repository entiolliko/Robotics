#include "ros/ros.h"
#include "project1/Reset.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "InitialPoseClient");
    if(argc != 4)
    {
        ROS_INFO("Usage: rosrun project set X Y THETA");
        return 1;
    }

    ros::NodeHandle n;

    ros::ServiceClient set_client = n.serviceClient<project1::Reset>("reset");

    project1::Reset srv;
    srv.request.reset_x = atoll(argv[1]);
    srv.request.reset_y = atoll(argv[2]);
    srv.request.reset_theta = atoll(argv[3]);

    if (set_client.call(srv))
    {
        ROS_INFO("Odometry has been set");
    }
    else
    {
        ROS_ERROR("Failed to call service set_odom");
        return 1;
    }
    return 0;
}
