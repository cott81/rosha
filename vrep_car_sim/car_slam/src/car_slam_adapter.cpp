/*
 * vrep_localizer_node.cpp
 *
 *  Created on: May 22, 2014
 *      Author: dominik
 */

#include "ros/ros.h"
#include "car_msgs/CarLaserScanData.h"
#include "car_msgs/InertiaSensorData.h"
#include "car_msgs/CarPose2D.h"
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
ros::Publisher* remote_InertiaDataPub;
ros::Publisher* remote_locGroundTruthPub;

double x = 0;
double y = 0;
double w = 0;

int robotId = 0; //robotId

void LaserScanDataCallback(const car_msgs::CarLaserScanData::ConstPtr& msg)
{
  if (robotId != msg->robotId)
  {
    // just process local msgs
    return;
  }

  //relay msg to remote topic
  remote_LaserScanPub->publish(msg);

  return;
}

void InertiaDataCallback(const car_msgs::InertiaSensorData::ConstPtr& msg)
{
  if (robotId != msg->robotId)
  {
    // just process local msgs
    return;
  }

  //relay msg to remote topic
  remote_InertiaDataPub->publish(msg);

  return;
}

void SimLocalizationCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  // relay msg to remote topic
  remote_locGroundTruthPub->publish(msg);

  return;
}

void RemoteLocalizationResultCallback(const car_msgs::CarPose2D::ConstPtr& msg)
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

  cout << "start car_slam_adapter" << endl;

  if (!robotIdByArg)
  {
    robotId = supplementary::SystemConfig::GetOwnRobotID();
  }

  string nodeName;
  stringstream sss;
  sss << "car_slam_adapter_node__" << robotId;
  sss >> nodeName;

  ros::init(argc, argv, nodeName);
  ros::NodeHandle n;


  ROS_INFO("own robot Id: %d\n", robotId);

  //error_seeder::ErrorSeederLib esl(compId);

  // build topic name
  string locResultTopic;
  string laserScanSubTopic;
  string inertiaDataTopic;
  string locGroundTruthSubTopic;
  stringstream ss;
  if (useRobotIdInTopic) {
    ss << "/vrep/carSim" << robotId << "/localizationInfo";
    ss >> locResultTopic;

    ss.str("");
    ss.clear();
    ss << "/vrep/carSim" << robotId << "/LaserScanData";
    ss >> laserScanSubTopic;

    ss.str("");
    ss.clear();
    ss << "/vrep/carSim" << robotId << "/localizationData";
    ss >> locGroundTruthSubTopic;

    ss.str("");
    ss.clear();
    ss << "/vrep/carSim" << robotId << "/InertiaData";
    ss >> inertiaDataTopic;
  }
  else
  {
    locResultTopic = "/vrep/carSim/localizationInfo";
   laserScanSubTopic = "/vrep/carSim/LaserScanData";
   inertiaDataTopic = "/vrep/carSim/InertiaData";
    //locGroundTruthSubTopic = "/vrep/MagicCube/localizationData";
    ROS_WARN(" ... use still robotId in interface topics to vrep simulator for simplicity!");
    ss << "/vrep/carSim" << robotId << "/localizationData";
    ss >> locGroundTruthSubTopic;
  }


  // localization result relay
  ros::Publisher pub = n.advertise<car_msgs::CarPose2D>( locResultTopic, 1000);
  locPublisher = &pub;
  string remote_locResultSubTopic = locResultTopic+"_REMOTE";
  ros::Subscriber remote_locResultSub = n.subscribe(remote_locResultSubTopic, 1000, RemoteLocalizationResultCallback);

  // laser data remote relay
  ros::Subscriber motionCmdSub = n.subscribe(laserScanSubTopic, 1000, LaserScanDataCallback);
  string remote_laserScanPubTopic = laserScanSubTopic+"_REMOTE";
  ros::Publisher  temp_pub1 = n.advertise<car_msgs::CarLaserScanData>( remote_laserScanPubTopic, 1000);
  remote_LaserScanPub = &temp_pub1;

  // inertia data remote relay
  ros::Subscriber inertiaDataSub = n.subscribe(inertiaDataTopic, 1000, InertiaDataCallback);
  string remote_inertiaDataTopic = inertiaDataTopic+"_REMOTE";
  ros::Publisher  temp_pub11 = n.advertise<car_msgs::InertiaSensorData>( remote_inertiaDataTopic, 1000);
  remote_InertiaDataPub = &temp_pub11;

  // receive ground truth loc
  ros::Subscriber locGroundTruthSub = n.subscribe(locGroundTruthSubTopic, 1000, SimLocalizationCallback);
  string remote_locGroundTruthPubTopic = locGroundTruthSubTopic+"_REMOTE";
  ros::Publisher temp_pub2 = n.advertise<geometry_msgs::PoseStamped>( remote_locGroundTruthPubTopic, 1000);
  remote_locGroundTruthPub = &temp_pub2;


  try
  {
    ros::Rate pub_rate(100.0);
    while (ros::ok())
    {
      ros::spinOnce();

      cout << "relay loop"<<endl;

      pub_rate.sleep();
    }
  }
  catch (exception& e)
  {
    ROS_FATAL("car_slam_adapter_node node caught exception. Aborting. %s", e.what());
    ROS_BREAK();
  }




}




