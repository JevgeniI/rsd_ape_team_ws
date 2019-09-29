#include <iostream>
#include "ros/ros.h"
#include "tf2_msgs/TFMessage.h"
#include "sensor_msgs/JointState.h"





void js_cb(const sensor_msgs::JointStateConstPtr& js_msg)
{
    std::cout << js_msg->position.at(0) << std::endl;
    ros::Duration(2).sleep();
}


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "ur_test_subscriber");
    ros::NodeHandle n;
    
    ros::Subscriber js_sub = n.subscribe("/joint_states", 1, js_cb);

    ros::spin();

    return 0;
}