/*
 * car_localizer_node.cpp
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

ros::Publisher* locPublisher;

double x = 0;
double y = 0;
double w = 0;

int robotId = 0; //robotId

void LaserScanDataCallback(const car_msgs::CarLaserScanData::ConstPtr& msg)
{
  if (robotId != msg->robotId)
  {
    // just process local msgs
    cout << "non local laser scan msg. msg robotId: " << msg->robotId << " != " << "own robotID:"<< robotId << std::endl;
    return;
  }

  // eliminated this output because it makes problems in the diagnosis
  //cout << "get laser scan data" << endl;

  //to some processing ... dummy ... simple use the loc data received directly from the sim

  //send position
  car_msgs::CarPose2D msg2;
  msg2.robotId = robotId;
  msg2.x = x;
  msg2.y = y;
  msg2.w = w;

  locPublisher->publish(msg2);

  return;
}

void InertiaDataCallback(const car_msgs::InertiaSensorData::ConstPtr& msg)
{
  if (robotId != msg->robotId)
  {
    // just process local msgs
    cout << "non local inertia data msg. msg robotId: " << msg->robotId << " != " << "own robotID:"<< robotId << std::endl;
    return;
  }

  // ... same as above
  //cout << "get inertia data data" << endl;

  //to some processing ... dummy ... simple use the loc data received directly from the sim

  return;
}

void SimLocalizationCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  //cout << "get pose info from vrep" << endl;

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
  string help = "Car SLAM\n"
      "Synobsis: car_slam_node OPTIONS\n"
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
    cout << argv[i] << " ";
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

  cout << "start car_slam" << endl;

  if (!robotIdByArg)
  {
    robotId = supplementary::SystemConfig::GetOwnRobotID();
  }

  string nodeName;
  stringstream sss;
  sss << "car_slam_node__" << robotId;
  sss >> nodeName;

  ros::init(argc, argv, nodeName);
  ros::NodeHandle n;


  ROS_INFO("own robot Id: %d\n", robotId);

  error_seeder::ErrorSeederLib esl(compId);

  // build topic name
  string locResultTopic;
  string motionCmdSubTopic;
  string locGroundTruthSubTopic;
  string inertiaDataTopic;
  stringstream ss;
  if (useRobotIdInTopic) {
    ss << "/vrep/carSim" << robotId << "/localizationInfo";
    ss >> locResultTopic;

    ss.str("");
    ss.clear();
    ss << "/vrep/carSim" << robotId << "/LaserScanData";
    ss >> motionCmdSubTopic;

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
    motionCmdSubTopic = "/vrep/carSim/LaserScanData";
    inertiaDataTopic = "/vrep/carSim/InertiaData";

    //locGroundTruthSubTopic = "/vrep/MagicCube/localizationData";
    ROS_WARN(" ... use still robotId in interface topics to vrep simulator for simplicity!");
    ss << "/vrep/carSim" << robotId << "/localizationData";
    ss >> locGroundTruthSubTopic;
  }


  ros::Publisher pub = n.advertise<car_msgs::CarPose2D>( locResultTopic, 1000);
  locPublisher = &pub;

  ros::Subscriber motionCmdSub = n.subscribe(motionCmdSubTopic, 1000, LaserScanDataCallback);

  ros::Subscriber locGroundTruthSub = n.subscribe(locGroundTruthSubTopic, 1000, SimLocalizationCallback);

  ros::Subscriber inertiaDataSub = n.subscribe(inertiaDataTopic, 1000, InertiaDataCallback);


  try
  {
    ros::Rate pub_rate(100.0);
    while (ros::ok())
    {
      ros::spinOnce();

      //TODO: heart beat msg
      cout << "slam loop"<<endl;

      pub_rate.sleep();
    }
  }
  catch (exception& e)
  {
    ROS_FATAL("car_slam_node node caught exception. Aborting. %s", e.what());
    ROS_BREAK();
  }




}




