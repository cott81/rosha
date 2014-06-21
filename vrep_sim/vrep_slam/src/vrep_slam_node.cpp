/*
 * vrep_localizer_node.cpp
 *
 *  Created on: May 22, 2014
 *      Author: dominik
 */

#include <vrep_slam/VrepSLAM.h>

#include "ros/ros.h"
#include "vrep_msgs/LaserScanData.h"
#include "vrep_msgs/Pose2D.h"
#include "std_msgs/Float64.h"
#include "error_seeder/ErrorSeederLib.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf/tf.h"
#include "SystemConfig.h"

#include <exception>

using namespace std;
using namespace vrep_slam;

VrepSLAM* vrepSLAM;
ros::Publisher* locPublisher;

double x = 0;
double y = 0;
double w = 0;

int robotId = 0; //robotId

void LaserScanDataCallback(const vrep_msgs::LaserScanData::ConstPtr& msg)
{
  if (robotId != msg->robotId)
  {
    // just process local msgs
    return;
  }
  cout << "get laser scan data" << endl;

  //to some processing ... dummy ... simple use the loc data received directly from the sim

  //send position
  vrep_msgs::Pose2D msg2;
  msg2.robotId = robotId;
  msg2.x = x;
  msg2.y = y;
  msg2.w = w;

  locPublisher->publish(msg2);

  return;
}

void SimLocalizationCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  //cout << "get pose" << endl;

  //convert to a 2D pose
  tf::Pose pose;
  tf::poseMsgToTF(msg->pose, pose);

  double yaw_angle = tf::getYaw(pose.getRotation());
  //cout << "\tyaw: " << yaw_angle << endl;

  x = msg->pose.position.x;
  y = msg->pose.position.y;
  w = yaw_angle;

  return;
}


int main (int argc, char** argv)
{
  bool robotIdByArg = false;
  bool useRobotIdInTopic = false;
  string help = "Vrep SLAM\n"
      "Synobsis: vrep_slam_node OPTIONS\n"
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
  int compId = -1;
  for (int i=1; i<argc; i++)
  {
    if(helpParam.compare(argv[i]) == 0)
    {
    cout << help << endl;
    exit(0);
    }
    else if (compIdParam.compare(argv[i]) == 0)
    {
      compId = atoi(argv[i+1]);
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

  cout << "start v_rep_slam" << endl;
  //bring the lib in
  vrepSLAM = new VrepSLAM();

  ros::init(argc, argv, "vrep_slam_node");
  ros::NodeHandle n;

  if (!robotIdByArg)
  {
    robotId = supplementary::SystemConfig::GetOwnRobotID();
  }
  ROS_INFO("own robot Id: %d\n", robotId);

  error_seeder::ErrorSeederLib esl(compId);

  // build topic name
  string locResultTopic;
  string motionCmdSubTopic;
  string locGroundTruthSubTopic;
  stringstream ss;
  if (useRobotIdInTopic) {
    ss << "/vrep/MagicCube" << robotId << "/localizationInfo";
    ss >> locResultTopic;

    ss.str("");
    ss.clear();
    ss << "/vrep/MagicCube" << robotId << "/LaserScanData";
    ss >> motionCmdSubTopic;

    ss.str("");
    ss.clear();
    ss << "/vrep/MagicCube" << robotId << "/localizationData";
    ss >> locGroundTruthSubTopic;
  }
  else
  {
    locResultTopic = "/vrep/MagicCube/localizationInfo";
    motionCmdSubTopic = "/vrep/MagicCube/LaserScanData";

    //locGroundTruthSubTopic = "/vrep/MagicCube/localizationData";
    ROS_WARN(" ... use still robotId in interface topics to vrep simulator for simplicity!");
    ss << "/vrep/MagicCube" << robotId << "/localizationData";
    ss >> locGroundTruthSubTopic;
  }

  ros::Publisher pub = n.advertise<vrep_msgs::Pose2D>( locResultTopic, 1000);
  locPublisher = &pub;

  ros::Subscriber motionCmdSub = n.subscribe(motionCmdSubTopic, 1000, LaserScanDataCallback);

  ros::Subscriber locGroundTruthSub = n.subscribe(locGroundTruthSubTopic, 1000, SimLocalizationCallback);


  try
  {
    ros::Rate pub_rate(30.0);
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
    ROS_FATAL("vrep_slam_node node caught exception. Aborting. %s", e.what());
    ROS_BREAK();
  }




}




