#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include <iostream>

int main(int argc, char** argv)
{

	//Initialize the arm_mover node
	ros::init(argc,argv,"simple_move");

	//Create a handle for the arm_mover node
	ros::NodeHandle n;

	//Create a Publisher object that can publish msg of type std_msgs::Float64 on topic "/simple_arm/joint_1_position_controller/command with queue size 10
	ros::Publisher joint_1_pub = n.advertise<std_msgs::Float64>
								("/simple_arm/joint_1_position_controller/command",10);


	//Publisher object to publish message of type std_msgs::Float64 on topic "/simple_arm/	joint_2_position_controller/command with queue size 10
	ros::Publisher joint_2_pub = n.advertise<std_msgs::Float64>
								("/simple_arm/joint_2_position_controller/command",10);


	//Set loop frequency 10 Hz
	ros::Rate loop_rate(10);

	int start_time=0, elapsed_time=0;
	//Get ROS start time
	while(not start_time){
		start_time = ros::Time::now().toSec();
	}


	//Start loop
	while(ros::ok())
	{
		
		//std::cout<<"Working"<<std::endl;
		//Get ROS elapsed Time
		elapsed_time = ros::Time::now().toSec() - start_time;

		//Set the arm joint angles
		std_msgs::Float64 joint_1_angle, joint_2_angle;
		joint_1_angle.data = sin(2 * M_PI * 0.1 * elapsed_time) * (M_PI/2);
		joint_2_angle.data = sin(2 * M_PI * 0.1 * elapsed_time) * (M_PI/2);

		//Publish the arm joint angles
		joint_1_pub.publish(joint_1_angle);
		joint_2_pub.publish(joint_2_angle);
		
		//Sleep till 10 Hz is over
		loop_rate.sleep();

	}
	//Close loop

	return 0;

}
