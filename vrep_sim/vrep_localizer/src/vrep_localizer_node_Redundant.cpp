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

#include <exception>

using namespace std;
using namespace vrep_localizer;

VrepLocalizer* vrepLoc;
ros::Publisher* locPublisher;
ros::Publisher* locPublisher2;


void SimLocalizationCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{

  cout << "get pose" << endl;

  tf::Pose pose;
  tf::poseMsgToTF(msg->pose, pose);

  double yaw_angle = tf::getYaw(pose.getRotation());
  cout << "\tyaw: " << yaw_angle << endl;

  //publish msg with "resulting" localiation infos
  vrep_msgs::Pose2D msg2;
  msg2.x = msg->pose.position.x;
  msg2.y = msg->pose.position.y;
  msg2.w = yaw_angle;

  locPublisher->publish(msg2);

  /*
  int stateId = msg->data;
    int actionId = recM->GetAction(stateId);

    std_msgs::String actionMsg;
    actionMsg.data = "test";
    actionPublisher->publish(actionMsg);
    */
}

void SimLocalizationCallback2(const geometry_msgs::PoseStamped::ConstPtr& msg)
{

  cout << "get pose" << endl;

  tf::Pose pose;
  tf::poseMsgToTF(msg->pose, pose);

  double yaw_angle = tf::getYaw(pose.getRotation());
  cout << "\tyaw: " << yaw_angle << endl;

  //publish msg with "resulting" localiation infos
  vrep_msgs::Pose2D msg2;
  msg2.x = msg->pose.position.x;
  msg2.y = msg->pose.position.y;
  msg2.w = yaw_angle;

  locPublisher2->publish(msg2);
}



int main (int argc, char** argv)
{
  //argument handling
  string help = "Vrep Localizer\n"
       "Synobsis: vrep_localizer_node OPTIONS\n"
       "Options:\n\n"
       "ROS params: paramter for ROS, check the ROS wiki for more details.\n"
       "-help: prints this help text\n"
       "-compId: specifies the Id of this component. (Used for failure simulation). Default is -1.\n"
       ;

   string helpParam = "-help";
   string compIdParam = "-compId";
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

  error_seeder::ErrorSeederLib esl(ownId);

  //msg ...
  ros::Publisher pub = n.advertise<vrep_msgs::Pose2D>("/vrep/MagicCubeXY/localizationInfo", 1000);
  locPublisher = &pub;

  ros::Subscriber locDataSub = n.subscribe("/vrep/MagicCubeXY/localizationData", 1000, SimLocalizationCallback);

  // use command line remapping of ROS: executable origTopic:=/namespace/newTopic

  try
  {
    ros::Rate pub_rate(30.0);
    while (ros::ok())
    {
      ros::spinOnce();
      //do something
      cout << "REDUNDANT loc loop" << endl;

      pub_rate.sleep();
    }
  }
  catch (exception& e)
  {
    ROS_FATAL("Recovery Manager node caught exception. Aborting. %s", e.what());
    ROS_BREAK();
  }




}




