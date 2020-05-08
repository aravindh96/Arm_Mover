//Include Files
#include "ros/ros.h"
#include "simple_arm/GoToPosition.h"
#include "std_msgs/Float64.h"
#include <string>
#include <sstream>


//Global Publisher variables
ros::Publisher joint1_pub, joint2_pub;

//Function to set the clamped values
std::vector<float> clamped_angles (float joint_1_req , float joint_2_req)
{
    //Define clamped angles
    float clamped1 = joint_1_req;
    float clamped2 = joint_2_req;

    //Get min max values from the paramters in Rosmaster
    float min_1 , max_1, min_2, max_2;

    //Create new node handle
    ros::NodeHandle n2;

    //Get Node Name
    std::string node_name = ros::this_node::getName();

    //Get min and max values
    n2.getParam(node_name + "/min_joint_1_angle",min_1);
    n2.getParam(node_name + "/max_joint_1_angle",max_1);
    n2.getParam(node_name + "/min_joint_2_angle",min_2);
    n2.getParam(node_name + "/max_joint_2_angle",max_2);

    //Check if joint1 satisfies confition and set the clamoped value
    if(clamped1 > max_1 || clamped1 < min_1){
        clamped1 = std::max(std::min(clamped1,max_1),min_1);
        ROS_WARN("Joint 1 Angle is out of the range(%1.2f,%1.2f), Angle set to: %1.2f",min_1,max_1,clamped1);
    }

    //Check if joint2 satisfies the condition and set clamped values
    if(clamped2 > max_2 || clamped2 < min_2){
        clamped2 = std::max(std::min(clamped2,max_2),min_2);
        ROS_WARN("Joint 2 out of bounds(%1.2f,%1.2f), Setting Angle to : %1.2f",min_2,max_2,clamped2);
    }
    //Store clamped data and return
    std::vector<float> clamped;
    clamped.push_back(clamped1);
    clamped.push_back(clamped2);
    
    return clamped;

}


//Callback function to handle Service Call to safe_move
bool handle_safe_move_service(simple_arm::GoToPosition::Request& req , simple_arm::GoToPosition::Response& res)
{
    //Request received msg
    ROS_INFO("GoTOPosition Request Received: %1.2f, %1.2f", (float)req.joint_1_position, (float)req.joint_2_position);

    //Check if request id within bounds call function
    std::vector<float> joint_angles = clamped_angles(req.joint_1_position,req.joint_2_position);

    //Create Float64 objevtts and save clamed angles data to the objects
    std_msgs::Float64 joint_1, joint_2;
    joint_1.data = joint_angles[0];
    joint_2.data = joint_angles[1];

    //Use global publish variables to publish the joint angles
    joint1_pub.publish(joint_1);
    joint2_pub.publish(joint_2);

    res.msg_feedback = "Joint Angles set to j1:";

    ROS_INFO_STREAM(res.msg_feedback);
    //Wait for 3 seconds for robot to settle
    ros::Duration(3).sleep();

    //return true
    return true;
}

//main function
int main(int argc , char** argv)
{

    //initialize the node
    ros::init(argc,argv,"arm_mover");

    //Create node handler
    ros::NodeHandle n1;

    //Set Publisher variables to publish on the topics
    joint1_pub = n1.advertise<std_msgs::Float64>("/simple_arm/joint_1_position_controller/command",10);
    joint2_pub = n1.advertise<std_msgs::Float64>("/simple_arm/joint_2_position_controller/command",10);

    // Create the Ros Service Server
    ros::ServiceServer serv = n1.advertiseService("/arm_mover/safe_move", handle_safe_move_service);

    //Handle Ros communication events
    
    ROS_INFO("Ready to set joint angles");
    ros::spin();
    //return
    return 0;
}
