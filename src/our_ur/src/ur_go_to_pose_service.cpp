#include <iostream>
#include "ros/ros.h"
#include "tf2_msgs/TFMessage.h"
#include "sensor_msgs/JointState.h"


// header: 
//   seq: 5808
//   stamp: 
//     secs: 116
//     nsecs: 637000000
//   frame_id: ''
// name: [elbow_joint, shoulder_lift_joint, shoulder_pan_joint, wrist_1_joint, wrist_2_joint,
//   wrist_3_joint]
// position: [-0.19001557127836577, -0.01820201321118109, -2.116860558275735, 2.519306476513375, -1.2432481661938581, 0.6940674480832758]
// velocity: [3.733951873148046e-05, 0.010240851904875278, -0.0028674483713320978, 0.0002311812819836747, 0.00010993670431971121, -3.7436135180699166e-05]


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