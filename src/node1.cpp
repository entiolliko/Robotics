#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include "geometry_msgs/PoseStamped.h"

class node1{



private :
	ros::NodeHandle n;
	ros::Subscriber pose_listener ;
	float x ;	

public :

	node1(){


		this->pose_listener = this-> n.subscribe("/robot/pose",1000,&node1::poseCallback,this);
		this->x = 0;
		    }

	
	
	void main_loop(){

		ros::Rate loop_rate(10);
		while(ros::ok())
		{
			ros::spinOnce();
		
		}



	}

	void  wheelCallback()
	{


	}

	void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
	{
		ROS_INFO("I heard [%f]",msg->pose.position.x);
	}


};

int main(int argc, char** argv)
{

	ros::init(argc,argv,"node1");
	
	node1 test_node;
	test_node.main_loop();
   //	ros::Publisher cmd_vel = n.advertise<geometry_msgs::Twist>("cmd_vel",1000);
}
