/*
 * vrep_inertia_node.cpp
 *
 *  Created on: April 8, 2014
 *      Author: dominik
 */


#include "ros/ros.h"
#include "vrep_msgs/LaserScanData.h"
#include "car_msgs/InertiaSensorData.h"
#include "std_msgs/Float64.h"
#include "error_seeder/ErrorSeederLib.h"

#include "SystemConfig.h"

#include <exception>

using namespace std;

ros::Publisher* inertiaDataPublisher;

int robotId = 0; //robotId

void SimInertiaCallback(const std_msgs::Float64& msg)
{
  /*
  if (robotId != msg->robotId)
  {
    // just process local msgs
    cout << "non local inertia data msg. msg robotId: " << msg->robotId << " != " << "own robotID:"<< robotId << std::endl;
    return;
  }
  */

  double data = msg.data;
  //cout << "sim data receive: " << data << endl;

  //some dummy data processing
  double dummyData = 0.0;
  for (int ii=0; ii<1000; ii++)
  {
    dummyData = (3 + ii) / dummyData;
  }

  // ... same as above
  //cout << "get inertia data data" << endl;

  //to some processing ... dummy ... simple use the loc data received directly from the sim

  return;
}

int main (int argc, char** argv)
{
  bool robotIdByArg = false;
  bool useRobotIdInTopic = false;
  string help = "Car Inertia Driver\n"
      "Synobsis: car_inertia_node OPTIONS\n"
      "Options:\n\n"
      "ROS params: paramter for ROS, check the ROS wiki for more details.\n"
      "-help: prints this help text\n"
      "-compId: specifies the Id of this component. (Used for failure simulation). Default is -1.\n"
      "-useRobotIdInTopic: code the robotId in the topic. E.g. /vrep/carSim12/localizationInfo instead of /vrep/MagicCube/localizationInfo. Default is false. \n"
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

  cout << "start car_inertia" << endl;

  if (!robotIdByArg)
  {
    robotId = supplementary::SystemConfig::GetOwnRobotID();
  }

  string nodeName;
  stringstream sss;
  sss << "car_inertia_node__" << robotId;
  sss >> nodeName;

  ros::init(argc, argv, nodeName);
  ros::NodeHandle n;

  ROS_INFO("own robot Id: %d\n", robotId);

  error_seeder::ErrorSeederLib esl(ownId);

  // build topic name
  string pubTopic;
  string simInertiaDataSubTopic;
  stringstream ss;
  if (useRobotIdInTopic) {
    ss << "/vrep/carSim" << robotId << "/InertiaData";
    ss >> pubTopic;

    ss.str("");
    ss.clear();
    ss << "/vrep/carSim" << robotId << "/SimInertiaData";
    ss >> simInertiaDataSubTopic;
  }
  else
  {
    pubTopic = "/vrep/carSim/InertiaData";
    // no interface to vrep yet, just provide some dummy data (see main loop)
    ROS_WARN(" ... use still robotId in interface topics to vrep simulator for simplicity!");
    ss << "/vrep/carSim" << robotId << "/SimInertiaData";
    ss >> simInertiaDataSubTopic;
  }


  ros::Publisher pub = n.advertise<car_msgs::InertiaSensorData>(pubTopic, 1000);
  inertiaDataPublisher = &pub;

  ros::Subscriber locDataSub = n.subscribe(simInertiaDataSubTopic, 1000, SimInertiaCallback);

  //ros::Subscriber motionCmdSub = n.subscribe("/vrep/MagicCubeXY/LaserScanDataFromSimulator", 1000, SimLaserDataCmdCallback);

  try
  {
    ros::Rate pub_rate(100.0);
    while (ros::ok())
    {
      ros::spinOnce();

      //TODO: heart beat msg
      cout << "inertia loop" << endl;


      //some dummy data processing
      double dummyData = 0.0;
      for (int ii=0; ii<100; ii++)
      {
        dummyData = (3 + ii) / dummyData;
      }

      //instead forwarding/ processing/ noising/ data from the simulator
      car_msgs::InertiaSensorData inertiaDataMsg;

      inertiaDataMsg.robotId = robotId;
      inertiaDataMsg.a_x = 0.0;
      inertiaDataMsg.a_y = 0.0;
      inertiaDataMsg.a_z = 0.0;
      inertiaDataPublisher->publish(inertiaDataMsg);

      pub_rate.sleep();
    }
  }
  catch (exception& e)
  {
    ROS_FATAL("vrep_inertia_node node caught exception. Aborting. %s", e.what());
    ROS_BREAK();
  }


}




