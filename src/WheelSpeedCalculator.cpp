#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/TwistStamped.h"
#include "project1/speeds.h"

#include <sstream>
#include <std_msgs/Float64.h>
#include <vector>

#include "data.h"

class WheelSpeedCalculator{

private :
	ros::NodeHandle n;
	ros::Subscriber robot_speed_listener;
	ros::Publisher wheel_vel_publisher;

	//create a message to read both velocity and position vectors
	project1::speeds out_msg;

	bool flag; //TODO: Forse è da togliere se riusciamo a sincronizzare i messaggi.


public :
	WheelSpeedCalculator(){
		this->robot_speed_listener = this-> n.subscribe("/cmd_vel",1000,&WheelSpeedCalculator::robot_speed_listenerCallback,this);
		this->wheel_vel_publisher = this-> n.advertise<project1::speeds>("wheels_rpm",1000);

		flag = false;

	}


	void main_loop(){
		ros::Rate loop_rate(10);

		while(ros::ok()){
			if(flag){
				this->wheel_vel_publisher.publish(out_msg);
				ROS_INFO("Ho pubblicato Questi Dati");
			  ROS_INFO("Front Left vale %f",out_msg.rpm_fl);
			  ROS_INFO("Front Right %f",out_msg.rpm_fr);
        ROS_INFO("Rear Left %f",out_msg.rpm_rl);
				ROS_INFO("Rear Right %f",out_msg.rpm_rr);

				this->flag = false;
			}
			ros::spinOnce();
		}
	}

	void robot_speed_listenerCallback(const geometry_msgs::TwistStamped::ConstPtr& in_msg){
		out_msg.rpm_fl = (1/R)*(in_msg->twist.linear.x - in_msg->twist.linear.y - (W+L)*in_msg->twist.angular.z) * (60) * (GEAR_RATIO);
    out_msg.rpm_fr = (1/R)*(in_msg->twist.linear.x + in_msg->twist.linear.y + (W+L)*in_msg->twist.angular.z) * (60) * (GEAR_RATIO);
    out_msg.rpm_rr = (1/R)*(in_msg->twist.linear.x - in_msg->twist.linear.y + (W+L)*in_msg->twist.angular.z) * (60) * (GEAR_RATIO);
    out_msg.rpm_rl = (1/R)*(in_msg->twist.linear.x + in_msg->twist.linear.y - (W+L)*in_msg->twist.angular.z) * (60) * (GEAR_RATIO);

    out_msg.header.seq = in_msg->header.seq;
		out_msg.header.stamp = in_msg->header.stamp;
		out_msg.header.frame_id = in_msg->header.frame_id;

		this-> flag = true;
	}

};

int main(int argc, char** argv){
	ros::init(argc,argv,"WheelSpeedCalculator");

	WheelSpeedCalculator test_node;
	test_node.main_loop();
}
