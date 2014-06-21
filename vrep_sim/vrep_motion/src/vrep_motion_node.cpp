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
#include "SystemConfig.h"

#include <exception>

using namespace std;
using namespace vrep_motion;

VrepMotion* vrepMotion;
ros::Publisher* leftWheelPublisher;
ros::Publisher* rightWheelPublisher;

int correctionFactor;
int robotId = 0; //robotId

void SimMotionCmdCallback(const vrep_msgs::DriveCmd::ConstPtr& msg)
{
  //cout << "get motion command" << endl;
  if (robotId != msg->robotId)
  {
    // just process local msgs
    return;
  }

  std_msgs::Float64 leftWheelMsg;
  std_msgs::Float64 rightWheelMsg;

  //convert ... NOT GENERAL just for this special case
  if (msg->rotV_w > 0)
  {
    // rotate counter clock wise
    //ROS_DEBUG("rotate counter clock wise");
    leftWheelMsg.data = msg->rotV_w * correctionFactor;
    rightWheelMsg.data = msg->rotV_w * correctionFactor * -1;
  }
  else if (msg->rotV_w < 0)
  {
    // rotate clock wise
    //ROS_DEBUG("rotate clock wise");
    leftWheelMsg.data = msg->rotV_w * correctionFactor; //switched order, because rotV_w is already negative
    rightWheelMsg.data = msg->rotV_w * correctionFactor * -1;
  }
  else
  {
    //drive straight
    //ROS_DEBUG("drive straight");
    leftWheelMsg.data = msg->transV_x * correctionFactor;
    rightWheelMsg.data = msg->transV_x * correctionFactor;
  }

  //send the msg
  leftWheelPublisher->publish(leftWheelMsg);
  rightWheelPublisher->publish(rightWheelMsg);
}


int main (int argc, char** argv)
{
  bool robotIdByArg = false;
  bool useRobotIdInTopic = false;
  string help = "Vrep Motion\n"
      "Synobsis: vrep_motion_node OPTIONS\n"
      "Options:\n\n"
      "ROS params: paramter for ROS, check the ROS wiki for more details.\n"
      "-help: prints this help text\n"
      "-compId: specifies the Id of this component. (Used for failure simulation). Default is -1.\n"
      "-useRobotIdInTopic: code the robotId in the topic. E.g. /vrep/MagicCube12/localizationInfo instead of /vrep/MagicCube/localizationInfo. Default is false. \n"
      "-robotId: specifies the Id of this system/robot. If not set, the config file (Global.conf, in the configuration path) is used. \n";

  string helpParam = "-help";
  string compIdParam = "-compId";
  string robotIdParam = "-robotId";
  string useRobotIdInTopicParam = "-useRobotIdInTopic";
  int ownId = -1;
  for (int i=1; i<argc; i++)
  {
    if(helpParam.compare(argv[i]) == 0)
    {
    cout << help << endl;
    exit(0);
    }
    else if (compIdParam.compare(argv[i]) == 0)
    {
      ownId = atoi(argv[i+1]);
    }
    else if (robotIdParam.compare(argv[i]) == 0)
    {
      robotId = atoi(argv[i+1]);
      robotIdByArg = true;
      //useRobotIdInTopic = true;
    }
    else if (useRobotIdInTopicParam.compare(argv[i]) == 0)
    {
      useRobotIdInTopic = true;
      ROS_WARN(" ... msg still contain the field \"robotId\". TODO: Create separate msg without that field! ");
    }
    else
    {
      //ros arguments ...
    }
  }

  cout << "start v_rep_motion" << endl;
  //bring the lib in
  vrepMotion = new VrepMotion();

  ros::init(argc, argv, "vrep_motion_node");
  ros::NodeHandle n;

  if (!robotIdByArg)
  {
    robotId = supplementary::SystemConfig::GetOwnRobotID();
  }
  ROS_INFO("own robot Id: %d\n", robotId);

  error_seeder::ErrorSeederLib esl(ownId);

  correctionFactor = -1; //magic cube in vrep wheel orientation twisted

  // build topic name
  string leftWheelpubTopic;
  string rightWheelpubTopic;
  string motionCmdSubTopic;
  stringstream ss;
  if (useRobotIdInTopic) {
    ss << "/vrep/MagicCube" << robotId << "/leftWheelVel";
    ss >> leftWheelpubTopic;

    ss.str("");
    ss.clear();
    ss << "/vrep/MagicCube" << robotId << "/rightWheelVel";
    ss >> rightWheelpubTopic;

    ss.str("");
    ss.clear();
    ss << "/vrep/MagicCube" << robotId << "/MotionCmd";
    ss >> motionCmdSubTopic;
  }
  else
  {
    //leftWheelpubTopic = "/vrep/MagicCube/leftWheelVel";
    ROS_WARN(" ... use still robotId in interface topics (leftWheelVel, rightWheelVel) to vrep simulator for simplicity!");
    ss << "/vrep/MagicCube" << robotId << "/leftWheelVel";
    ss >> leftWheelpubTopic;
    ss.str("");
    ss.clear();
    ss << "/vrep/MagicCube" << robotId << "/rightWheelVel";
    ss >> rightWheelpubTopic;
    //rightWheelpubTopic = "/vrep/MagicCube/rightWheelVel";

    motionCmdSubTopic = "/vrep/MagicCube/MotionCmd";
  }

  ros::Publisher pub = n.advertise<std_msgs::Float64>(leftWheelpubTopic, 1000);
  leftWheelPublisher = &pub;

  ros::Publisher pub2 = n.advertise<std_msgs::Float64>(rightWheelpubTopic, 1000);
  rightWheelPublisher = &pub2;

  ros::Subscriber motionCmdSub = n.subscribe(motionCmdSubTopic, 1000, SimMotionCmdCallback);

  //ros::Subscriber errorSub = n.subscribe("/ErrorSeeder/ErrorTrigger", 1000, ErrorTriggerCallback);

  // use command line remapping of ROS: executable origTopic:=/namespace/newTopic

  try
  {
    ros::Rate pub_rate(30.0);
    //ros::Rate pub_rate(1.0);
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




