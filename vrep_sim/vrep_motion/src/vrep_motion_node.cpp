/*
 * vrep_localizer_node.cpp
 *
 *  Created on: May 22, 2014
 *      Author: dominik
 */

#include "../include/vrep_motion/VrepMotion.h"

#include "ros/ros.h"
#include "vrep_msgs/DriveCmd.h"
#include "std_msgs/Float64.h"
//#include "error_seeder_msgs/Error.h"
#include "error_seeder/ErrorSeederLib.h"

#include <exception>

using namespace std;
using namespace vrep_motion;

VrepMotion* vrepMotion;
ros::Publisher* leftWheelPublisher;
ros::Publisher* rightWheelPublisher;

int correctionFactor;


void SimMotionCmdCallback(const vrep_msgs::DriveCmd::ConstPtr& msg)
{
  cout << "get motion command" << endl;

  std_msgs::Float64 leftWheelMsg;
  std_msgs::Float64 rightWheelMsg;

  //convert ... NOT GENERAL just for this special case
  if (msg->rotV_w > 0)
  {
    // rotate counter clock wise
    leftWheelMsg.data = msg->rotV_w * -1 * correctionFactor;
    rightWheelMsg.data = msg->rotV_w * correctionFactor;
  }
  else if (msg->rotV_w < 0)
  {
    // rotate clock wise
    leftWheelMsg.data = msg->rotV_w * correctionFactor;
    rightWheelMsg.data = msg->rotV_w * -1 * correctionFactor;
  }
  else
  {
    //drive straight
    leftWheelMsg.data = msg->transV_x * correctionFactor;
    rightWheelMsg.data = msg->transV_x * correctionFactor;
  }

  //send the msg
  leftWheelPublisher->publish(leftWheelMsg);
  rightWheelPublisher->publish(rightWheelMsg);
}

/*
void ErrorTriggerCallback(const error_seeder_msgs::Error::ConstPtr& msg)
{
  cout << "received msg .. " << endl;
}
*/


int main (int argc, char** argv)
{
  cout << "start v_rep_motion" << endl;
  //bring the lib in
  vrepMotion = new VrepMotion();

  ros::init(argc, argv, "vrep_motion_node");
  ros::NodeHandle n;

  int a[1] = {1};

  int b = a[25];
  cout << b << endl;

  int ownId = 1;
  error_seeder::ErrorSeederLib esl(ownId);

  correctionFactor = -1; //magic cube in vrep wheel orientation twisted

  ros::Publisher pub = n.advertise<std_msgs::Float64>("/vrep/MagicCubeXY/leftWheelVel", 1000);
  leftWheelPublisher = &pub;
  ros::Publisher pub2 = n.advertise<std_msgs::Float64>("/vrep/MagicCubeXY/rightWheelVel", 1000);
  rightWheelPublisher = &pub2;

  ros::Subscriber motionCmdSub = n.subscribe("/vrep/MagicCubeXY/MotionCmd", 1000, SimMotionCmdCallback);

  //ros::Subscriber errorSub = n.subscribe("/ErrorSeeder/ErrorTrigger", 1000, ErrorTriggerCallback);

  // use command line remapping of ROS: executable origTopic:=/namespace/newTopic

  try
  {
    //ros::Rate pub_rate(30.0);
    ros::Rate pub_rate(1.0);
    while (ros::ok())
    {
      ros::spinOnce();

      //TODO: heart beat msg
      cout << "loop"<<endl;

      pub_rate.sleep();
    }
  }
  catch (exception& e)
  {
    ROS_FATAL("vrep_motion_node node caught exception. Aborting. %s", e.what());
    ROS_BREAK();
  }




}




