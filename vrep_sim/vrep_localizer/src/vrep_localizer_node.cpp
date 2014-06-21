/*
 * vrep_localizer_node.cpp
 *
 *  Created on: May 22, 2014
 *      Author: dominik
 */

#include "../include/vrep_localizer/VrepLocalizer.h"

#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "vrep_msgs/Pose2D.h"
#include "tf/tf.h"

#include "error_seeder/ErrorSeederLib.h"

#include <SystemConfig.h>

#include <exception>

using namespace std;
using namespace vrep_localizer;

VrepLocalizer* vrepLoc;
ros::Publisher* locPublisher;

int robotId = 0; //robotId


void SimLocalizationCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{

  cout << "get pose" << endl;

  tf::Pose pose;
  tf::poseMsgToTF(msg->pose, pose);

  double yaw_angle = tf::getYaw(pose.getRotation());
  cout << "\tyaw: " << yaw_angle << endl;

  //publish msg with "resulting" localiation infos
  vrep_msgs::Pose2D msg2;
  msg2.robotId = robotId;
  msg2.x = msg->pose.position.x;
  msg2.y = msg->pose.position.y;
  msg2.w = yaw_angle;

  locPublisher->publish(msg2);
}



int main (int argc, char** argv)
{
  bool robotIdByArg = false;
  bool useRobotIdInTopic = false;
  //argument handling
  string help = "Vrep Localizer\n"
       "Synobsis: vrep_localizer_node OPTIONS\n"
       "Options:\n\n"
       "ROS params: paramter for ROS, check the ROS wiki for more details.\n"
       "-help: prints this help text\n"
       "-compId: specifies the Id of this component. (Used for failure simulation). Default is -1.\n"
      "-useRobotIdInTopic: code the robotId in the topic. E.g. /vrep/MagicCube12/localizationInfo instead of /vrep/MagicCube/localizationInfo. Default is false. \n"
      "-robotId: specifies the Id of this system/robot. If not set, the config file (Global.conf, in the configuration path) is used. \n"
       ;

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

  cout << "start v_rep_localizer" << endl;
  //bring the lib in
  vrepLoc = new VrepLocalizer();

  ros::init(argc, argv, "vrep_localizer_node");
  ros::NodeHandle n;

  if (!robotIdByArg)
  {
    robotId = supplementary::SystemConfig::GetOwnRobotID();
  }
  ROS_INFO("own robot Id: %d\n", robotId);

  error_seeder::ErrorSeederLib esl(ownId);

  // build topic name
  string locResultTopic;
  string locGroundTruthSubTopic;
  stringstream ss;
  if (useRobotIdInTopic) {
    ss << "/vrep/MagicCube" << robotId << "/localizationInfo";
    ss >> locResultTopic;

    ss.str("");
    ss.clear();
    ss << "/vrep/MagicCube" << robotId << "/localizationData";
    ss >> locGroundTruthSubTopic;
  }
  else
  {
    locResultTopic = "/vrep/MagicCube/localizationInfo";
    //locGroundTruthSubTopic = "/vrep/MagicCube/localizationData";
    ROS_WARN(" ... use still robotId in interface topics to vrep simulator for simplicity!");
    ss << "/vrep/MagicCube" << robotId << "/localizationData";
    ss >> locGroundTruthSubTopic;
  }


  ros::Publisher pub = n.advertise<vrep_msgs::Pose2D>(locResultTopic, 1000);
  locPublisher = &pub;

  ros::Subscriber locDataSub = n.subscribe(locGroundTruthSubTopic, 1000, SimLocalizationCallback);

  // use command line remapping of ROS: executable origTopic:=/namespace/newTopic

  try
  {
    ros::Rate pub_rate(30.0);
    while (ros::ok())
    {
      ros::spinOnce();
      //do something
      cout << "loc loop" << endl;
      /*
      cout << "send test msg" << endl;
        vrep_msgs::Pose2D msg2;
        msg2.x = 1.0;
        msg2.y = 2.0;
        msg2.w = 3.0;

        locPublisher->publish(msg2);
      */

      pub_rate.sleep();
    }
  }
  catch (exception& e)
  {
    ROS_FATAL("Recovery Manager node caught exception. Aborting. %s", e.what());
    ROS_BREAK();
  }




}




