//Include Files
#include "ros/ros.h"
#include "simple_arm/GoToPosition.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/JointState.h"

//Global Service Client Object
ros::ServiceClient client;
//Global joints_last_position vector and moving_state boolean
bool moving_state = false;
std::vector<double> joints_last_position{0,0};

//joint_states_callback - Checks whether the arm is moving or not and
// updates global moving_state boolean
void joint_states_callback(sensor_msgs::JointState js)
{
    std::vector<double> joints_curr_position = js.position;
    
    double tolerance = 0.0005;

    if(joints
}

//Main function
int main(int argc, char** argv)
{
    //Initializing the node
    ros::init(argc,argv,"look_away");

    //Creating a Node Handle
    ros::NodeHandle n;

    //Setting up the Service Client
    client = n.advertiseClient<simple_arm::GoToPosition>("/arm_mover/safe_move");

    //Subscribe to Joint State topic to get current position
    ros::Subscriber sub1 = n.subscribe("/simple_arm/joint_states",10,joint_states_callback);

    //Subscribe to Image_raw topic to get the images from camera
    ros::Subscriber sub2 = n.subscribe("/rgb_camera/image_raw", 10,look_away_callback);

    //Handle communications and wait for callbacks
    ros::spin();

    return 0;
}
