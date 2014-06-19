/*
 * vrep_localizer_node.cpp
 *
 *  Created on: May 22, 2014
 *      Author: dominik
 */

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

//publish to local
ros::Publisher* locPublisher;
//publish to remote
ros::Publisher* remote_LaserScanPub;
ros::Publisher* remote_motionCmdPub;

double x = 0;
double y = 0;
double w = 0;

int robotId = 0; //robotId

void LaserScanDataCallback(const vrep_msgs::LaserScanData::ConstPtr& msg)
{

  //relay msg to remote topic
  remote_LaserScanPub->publish(msg);

  return;
}

void SimLocalizationCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  // relay msg to remote topic
  remote_motionCmdPub->publish(msg);

  return;
}

void RemoteLocalizationResultCallback(const vrep_msgs::Pose2D::ConstPtr& msg)
{
  // relay msg to remote topic
  locPublisher->publish(msg);

  return;
}


int main (int argc, char** argv)
{
  bool robotIdByArg = false;
  bool useRobotIdInTopic = false;
  string help = "Vrep SLAM Adapter\n"
      "Synobsis: vrep_slam_adapter OPTIONS\n"
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
      useRobotIdInTopic = true;
    }
    else if (useRobotIdInTopicParam.compare(argv[i]) == 0)
    {
      useRobotIdInTopic = true;
    }
    else
    {
      //ros arguments ...
    }
  }

  cout << "start v_rep_slam_adapter" << endl;

  ros::init(argc, argv, "vrep_slam_adapter");
  ros::NodeHandle n;

  if (!robotIdByArg)
  {
    robotId = supplementary::SystemConfig::GetOwnRobotID();
  }
  ROS_INFO("own robot Id: %d\n", robotId);

  //error_seeder::ErrorSeederLib esl(compId);





  //CONTINUE HERE to integrate the robotId in topic switch ...





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
    locGroundTruthSubTopic = "/vrep/MagicCube/localizationData";
  }

  ros::Publisher pub = n.advertise<vrep_msgs::Pose2D>( locResultTopic, 1000);
  locPublisher = &pub;
  string remote_locResultSubTopic = locResultTopic+"_REMOTE";
  ros::Subscriber remote_locResultSub = n.subscribe(remote_locResultSubTopic, 1000, RemoteLocalizationResultCallback);

  ros::Subscriber motionCmdSub = n.subscribe(motionCmdSubTopic, 1000, LaserScanDataCallback);
  string remote_motionCmdSubTopic = motionCmdSubTopic+"_REMOTE";
  ros::Publisher  temp_pub1 = n.advertise<vrep_msgs::LaserScanData>( remote_motionCmdSubTopic, 1000);
  remote_motionCmdPub = &temp_pub1;

  ros::Subscriber locGroundTruthSub = n.subscribe(locGroundTruthSubTopic, 1000, SimLocalizationCallback);
  string remote_locGroundTruthSubTopic = locGroundTruthSubTopic+"_REMOTE";
  ros::Publisher temp_pub2 = n.advertise<geometry_msgs::PoseStamped>( remote_locGroundTruthSubTopic, 1000);
  remote_LaserScanPub = &temp_pub1;


  try
  {
    ros::Rate pub_rate(30.0);
    while (ros::ok())
    {
      ros::spinOnce();

      cout << "relay loop"<<endl;

      pub_rate.sleep();
    }
  }
  catch (exception& e)
  {
    ROS_FATAL("vrep_slam_node node caught exception. Aborting. %s", e.what());
    ROS_BREAK();
  }




}




