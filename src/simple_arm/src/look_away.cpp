//Include Files
#include "ros/ros.h"
#include "simple_arm/GoToPosition.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/JointState.h"
#include <vector>

//Global Service Client Object
ros::ServiceClient client;
//Global joints_last_position vector and moving_state boolean
auto moving_state = false;
// auto itr;
std::vector<double> joints_last_position;
joints_last_position.push_back(0);
joints_last_position.push_back(0);

//move_to_center - Moves the Arm to the center position
void move_to_center()
{
    simple_arm::GoToPosition joint_angle_srv;
    joint_angle_srv.request.joint_1_position = 1.57;
    joint_angle_srv.request.joint_2_position = 1.57;

    if(!client.call(joint_angle_srv)){
        ROS_ERROR("Failed to call service safe_move");
    }


}

//joint_states_callback - Checks whether the arm is moving or not and
// updates global moving_state boolean
void joint_states_callback(sensor_msgs::JointState js)
{
    std::vector<double> joints_curr_position = js.position;
    
    double tolerance = 0.0005;

    if(joints_curr_position[0] - joints_last_position[0] < tolerance ||
        joints_curr_position[1] - joints_last_position[1] < tolerance)
        {
            moving_state = false;
        }
        else{
            moving_state = true;
            joints_last_position = joints_curr_position;
        }
}

// look_away_callback - Checks if the image is blank, if so moves the 
// arm to the center position 
void look_away_callback(const sensor_msgs::Image img)
{
    bool uniform_image = true;

    for(int i=0; i<img.height * img.step; i++){
        if(img.data[i]-img.data[0] != 0){
            uniform_image = false;
            break;
        }
    }

    if(uniform_image == true && moving_state == false){
        move_to_center();
    }

}

//Main function
int main(int argc, char** argv)
{
    //Initializing the node
    ros::init(argc,argv,"look_away");

    //Creating a Node Handle
    ros::NodeHandle n;

    //Setting up the Service Client
    client = n.serviceClient<simple_arm::GoToPosition>("/arm_mover/safe_move");

    //Subscribe to Joint State topic to get current position
    ros::Subscriber sub1 = n.subscribe("/simple_arm/joint_states",10,joint_states_callback);

    //Subscribe to Image_raw topic to get the images from camera
    ros::Subscriber sub2 = n.subscribe("/rgb_camera/image_raw", 10,look_away_callback);

    //Handle communications and wait for callbacks
    ros::spin();

    return 0;
}
