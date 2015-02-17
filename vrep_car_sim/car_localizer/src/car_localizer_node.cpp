/*
 * car_localizer_node.cpp
 *
 *  Created on: May 22, 2014
 *      Author: dominik
 */

#include "../include/car_localizer/CarLocalizer.h"

#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "car_msgs/Pose2D.h"
#include "tf/tf.h"
#include <math.h>

#include "error_seeder/ErrorSeederLib.h"

#include <SystemConfig.h>

#include <exception>

using namespace std;
using namespace car_localizer;

CarLocalizer* carLoc;
ros::Publisher* locPublisher;

int robotId = 0; //robotId


void SimLocalizationCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{

  tf::Pose pose;
  tf::poseMsgToTF(msg->pose, pose);

  double yaw_angle = tf::getYaw(pose.getRotation());

  //publish msg with "resulting" localisation infos
  car_msgs::Pose2D msg2;
  msg2.robotId = robotId;
  msg2.x = msg->pose.position.x;
  msg2.y = msg->pose.position.y;
  msg2.w = yaw_angle;
//  double vecX = cos(yaw_angle);
//  double vecY = sin(yaw_angle);
//  cout << "X: " << vecX << endl;
//  cout << "Y: " << vecY << endl;
//  cout << "W: " << yaw_angle << endl;
//  cout << endl;
//  msg2.dirVecX = Ma
//  msg2.dirVecX = cos(yaw_angle);
//  msg2.dirVecX = sin(yaw_angle);


  locPublisher->publish(msg2);
}



int main (int argc, char** argv)
{
  bool robotIdByArg = false;
  bool useRobotIdInTopic = false;
  //argument handling
  string help = "Car Localizer\n"
       "Synobsis: car_localizer_node OPTIONS\n"
       "Options:\n\n"
       "ROS params: paramter for ROS, check the ROS wiki for more details.\n"
       "-help: prints this help text\n"
       "-compId: specifies the Id of this component. (Used for failure simulation). Default is -1.\n"
      "-useRobotIdInTopic: code the robotId in the topic. E.g. /vrep/carSim12/localizationInfo instead of /vrep/carSim/localizationInfo. Default is false. \n"
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
//       ROS_WARN(" ... msg still contain the field \"robotId\". TODO: Create separate msg without that field! ");
     }
     else
     {
       //ros arguments ...
     }
   }

  cout << "start car_localizer" << endl;
  //bring the lib in
  carLoc = new CarLocalizer();

  ros::init(argc, argv, "car_localizer_node");
  ros::NodeHandle n;

  if (!robotIdByArg)
  {
    robotId = supplementary::SystemConfig::GetOwnRobotID();
  }
  ROS_INFO("own robot Id: %d\n", robotId);

  error_seeder::ErrorSeederLib esl(ownId);

  // build topic name
  string locGroundTruthSubTopic;
  stringstream ss;
  if (useRobotIdInTopic) {
    ss << "/vrep/carSim" << robotId << "/localizationData";
    ss >> locGroundTruthSubTopic;
  }
  else
  {
    ss << "/vrep/carSim" << robotId << "/localizationData";
    ss >> locGroundTruthSubTopic;
  }


  ros::Publisher pub = n.advertise<car_msgs::Pose2D>("/vrep/carSim/localizationInfo", 1000);
  locPublisher = &pub;

  ros::Subscriber locDataSub = n.subscribe(locGroundTruthSubTopic, 1000, SimLocalizationCallback);

  // use command line remapping of ROS: executable origTopic:=/namespace/newTopic

  try
  {
    ros::Rate pub_rate(100.0);
    while (ros::ok())
    {
      ros::spinOnce();
      //do something
//      cout << "loc loop" << endl;
      /*
      cout << "send test msg" << endl;
        car_msgs::Pose2D msg2;
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




