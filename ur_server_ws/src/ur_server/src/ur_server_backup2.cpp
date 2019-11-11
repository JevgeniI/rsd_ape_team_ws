#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <ur_server/urAction.h>

#include <rtde_control_interface.h>
#include <rtde_receive_interface.h>
#include <thread> // only needed for the delay
#include <iostream>
#include <fstream>
#include <algorithm>
#include <string>
#include <iterator>
#include <vector>

using namespace ur_rtde;

//Connect to the robot:
RTDEControlInterface rtde_control("192.168.100.1");//("127.0.0.1");//
RTDEReceiveInterface rtde_receive("192.168.100.1");//("127.0.0.1");//

std::vector<std::vector<double>> last_path;
int last_index;
bool interrupted = false;


class urAction
{
protected:

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<ur_server::urAction> as_; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
  std::string action_name_;
  // create messages that are used to published feedback/result
  ur_server::urFeedback feedback_;
  ur_server::urResult result_;


public:

  urAction(std::string name) :
    as_(nh_, name, boost::bind(&urAction::execute, this, _1), false),
    action_name_(name)
  {
    as_.start();
  }

  ~urAction(void)
  {
  }

  void execute(const ur_server::urGoalConstPtr &goal)
  {
    // helper variables
    ros::Rate r(1);
    bool success = true;

    //std::cout << "Last_path  " << last_path << std::endl;
    std::cout << "Last_index  " << last_index << std::endl;
    std::cout << "Was interrupted?  " << interrupted << std::endl;


    float goaln = goal->order;
    std::cout << "Goal variable is" << goaln << std::endl;
    std::cout << "Init Goal variable is" << goal->order << std::endl;
    std::cout << "Main goal variable is" << goal << std::endl;


    //-////////////1) Read the path file: //////////////////////////////////////
    std::vector<std::vector<double>> full_path;
    std::string line;
    std::ifstream myfile;

    myfile.open("/home/jevgeni/Documents/ur_server_ws/paths/movetobrick.txt");
    long rows;
    for (rows = 0; std::getline(myfile, line,'\n'); ++rows);
    std::cout<<rows;
    myfile.close();

    myfile.open("/home/jevgeni/Documents/ur_server_ws/paths/movetobrick.txt");
    for (int i = 0; i < rows; i++)
      {
         std::vector<double> newRow;
         for (int j = 0; j < 6; j++)
         {
            double val;
            char c;
            myfile.get(c);
            myfile >> val;
            newRow.push_back(val);
         }
         full_path.push_back(newRow);
      }
    myfile.close();


    //-/////////////////////////////////////////////////////////////////

    //-/////////////2) Move robot to init position:
  /* rtde_control.servoJ({0.5, -1.5708, 3.32038e-05, -1.57255, 0.000772953, 8.05855e-05},  0.2,  0.2, 0.3, 0.1, 300);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    for(int i = 0; i < 20; i++){
        if (as_.isPreemptRequested() || !ros::ok()){
            ROS_INFO("%s: Preempted", action_name_.c_str());
            // set the action state to preempted
            as_.setPreempted();
            success = false;
            interrupted = true;
            break;
        } else {
            rtde_control.servoJ({0.5, -1.5708, 3.32038e-05, -1.57255, 0.000772953, 8.05855e-05},  0.2,  0.2, 0.8, 0.2, 300);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }*/

    // publish info to the console for the user
    ROS_INFO("Executing, moving robot to point");
    //-/////////////////////////////////////////////////////////////////
   // rtde_control.servoJ({0.5, -1.5708, 3.32038e-05, -1.57255, 0.000772953, 8.05855e-05},  0.2,  0.2, 0.3, 0.1, 300);
   // std::this_thread::sleep_for(std::chrono::milliseconds(1000));
   // rtde_control.servoStop();


    //-/////////////3) Move robot to next position(can be preempted)://///////////////////
    for(int i = 0; i < full_path.size(); i++){
        if (as_.isPreemptRequested() || !ros::ok())
        {
          ROS_INFO("%s: Preempted", action_name_.c_str());
          // set the action state to preempted
          as_.setPreempted();
          success = false;
          interrupted = true;
          last_path.clear();
          last_path = full_path;
          last_index = i;
          break;
        } else {
        rtde_control.servoJ({full_path[i]}, 0.2, 1.4, 0.4, 0.2, 300);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    rtde_control.servoStop();
    //-/////////////////////////////////////////////////////////////////

    ROS_INFO("EXECUTING SECOND MOVE");


    //-/////////////4) Get and print resulting pose:///////////////////
    std::vector<double> joint_positions = rtde_receive.getActualQ();
    std::vector<double> joint_target = rtde_receive.getTargetQ();

    std::cout << "Current positions: ";
    for(int i = 0; i < 6; i++){
      std::cout << joint_positions[i] << "   ";
    }

    std::cout << std::endl;

    std::cout << "Target positions: ";
    for(int i = 0; i < 6; i++){
      std::cout << joint_target[i] << "   ";
    }

    std::cout << std::endl;

    //-/////////////////////////////////////////////////////////////////


    //-/////////////5) Check for success:///////////////////

    if(success)
    {
      interrupted = false;
      result_.sequence = feedback_.sequence;
      ROS_INFO("%s: Succeeded", action_name_.c_str());
      // set the action state to succeeded
      as_.setSucceeded(result_);
    }
  }

};


int main(int argc, char** argv)
{

  ros::init(argc, argv, "fibonacci");
  system("pwd");

  urAction urcontrol("urcontrol");
  ros::spin();

  return 0;
}
