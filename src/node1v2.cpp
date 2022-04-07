#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"


class node1{



private :
	ros::NodeHandle n;
	ros::Subscriber pose_listener ;
	ros::Publisher vel_publisher;
	float x ;
	float y;
	float z;
	float x_quat;
	float y_quat;
	float z_quat;
	float w_quat;	
	geometry_msgs::TwistStamped out_msg;
	bool flag;

public :

	node1(){


		this->pose_listener = this-> n.subscribe("/robot/pose",1000,&node1::poseCallback,this);
		this->vel_publisher = this-> n.advertise<geometry_msgs/TwistStamped>("cmd_vel",1000);
		this->x =0;
		this->y =0;
		this->z =0;
		this->x_quat =0;
		this->y_quat =0;
                this->z_quat =0;
                this->w_quat =0;
		flag = false;
		    }

	
	
	void main_loop(){

		ros::Rate loop_rate(10);
		while(ros::ok())
		{
			if(flag)
			{
				this->vel_publisher.publish(out_msg);
				this->flag = false;
			}	
			ros::spinOnce();
		
		}



	}

	void  wheelCallback()
	{


	}

	void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& in_msg)
	{	
		//ROS_INFO("I heard [%f]",msg->pose.position.x);
		this->x = in_msg->pose.position.x;
		this->y = in_msg->pose.position.y;
		this->z = in_msg->pose.position.z;
                this->x_quat = in_msg->pose.orientation.x;
		this->y_quat = in_msg->pose.orientation.y;
                this->z_quat = in_msg->pose.orientation.z;
		this->w_quat = in_msg->pose.orientation.w;
		calculate_data();
		
	}

	void calculate_data()
	{
		this-> flag = true;
	}

	void calculate_euler()
	{


	}

	void calculate_rk()
	{


	}

};

int main(int argc, char** argv)
{

	ros::init(argc,argv,"node1");
	
	node1 test_node;
	test_node.main_loop();

}
