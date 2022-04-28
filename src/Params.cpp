#include "ros/ros.h"
#include <dynamic_reconfigure/server.h>
#include <project1/parametersConfig.h>
#include <std_msgs/Float64.h>

void callback(project1::parametersConfig &config, uint32_t bitmask_level)
{
    ROS_INFO("Reconfigure request: %d", config.odomMethod);
    ROS_INFO ("%d",bitmask_level);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "params");

    //Create the server and specify callback function type
    dynamic_reconfigure::Server<project1::parametersConfig> server;
    dynamic_reconfigure::Server<project1::parametersConfig>::CallbackType f;

    //Bind and set callback function
    f = boost::bind(&callback, _1, _2);
    server.setCallback(f);

    ros::spin();
    return 0;
}
