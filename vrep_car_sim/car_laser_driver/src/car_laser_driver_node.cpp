/*
 * vrep_localizer_node.cpp
 *
 *  Created on: May 22, 2014
 *      Author: dominik
 */


#include "ros/ros.h"
#include "car_msgs/CarLaserScanData.h"
#include "std_msgs/Float64.h"
#include "error_seeder/ErrorSeederLib.h"

#include "SystemConfig.h"

#include <exception>

using namespace std;

ros::Publisher* laserScanDataPublisher;

int robotId = 0; //robotId



void SimLaserDataCmdCallback(const std_msgs::Float64& msg)
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

  return;
}

int main (int argc, char** argv)
{
  bool robotIdByArg = false;
  bool useRobotIdInTopic = false;
  string help = "Car Laser Driver\n"
      "Synobsis: car_laser_driver_node OPTIONS\n"
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

  cout << "start car_laser_driver" << endl;

  if (!robotIdByArg)
  {
    robotId = supplementary::SystemConfig::GetOwnRobotID();
  }

  string nodeName;
  stringstream sss;
  sss << "car_laser_driver_node__" << robotId;
  sss >> nodeName;

  ros::init(argc, argv, nodeName);
  ros::NodeHandle n;

  ROS_INFO("own robot Id: %d\n", robotId);

  error_seeder::ErrorSeederLib esl(ownId);

  // build topic name
  string pubTopic;
  string simLaseDataSubTopic;
  stringstream ss;
  if (useRobotIdInTopic) {
    ss << "/vrep/carSim" << robotId << "/LaserScanData";
    ss >> pubTopic;

    ss.str("");
    ss.clear();
    ss << "/vrep/carSim" << robotId << "/SimInertiaData";
    ss >> simLaseDataSubTopic;

  }
  else
  {
    pubTopic = "/vrep/carSim/LaserScanData";

    // no interface to vrep yet, just provide some dummy data (see main loop)
    ROS_WARN(" ... use still robotId in interface topics to vrep simulator for simplicity!");
    ss << "/vrep/carSim" << robotId << "/SimInertiaData";
    ss >> simLaseDataSubTopic;
  }


  ros::Publisher pub = n.advertise<car_msgs::CarLaserScanData>(pubTopic, 1000);
  laserScanDataPublisher = &pub;

  ros::Subscriber simLaserDataSub = n.subscribe(simLaseDataSubTopic, 1000, SimLaserDataCmdCallback);

  //ros::Subscriber motionCmdSub = n.subscribe("/vrep/MagicCubeXY/LaserScanDataFromSimulator", 1000, SimLaserDataCmdCallback);

  try
  {
    ros::Rate pub_rate(100.0);
    while (ros::ok())
    {
      ros::spinOnce();

      //TODO: heart beat msg
      cout << "laser loop" << endl;

      //instead forwarding/ processing/ noising/ data from the simulator

      //some dummy data processing
      double dummyData = 0.0;
      for (int ii=0; ii<100; ii++)
      {
        dummyData = (3 + ii) / dummyData;
      }

      car_msgs::CarLaserScanData msgLaserScan;
      msgLaserScan.robotId = robotId;

      for (unsigned int i = 0; i < 100; i++)
      {
        msgLaserScan.laserScanData.push_back(12);

      }
      laserScanDataPublisher->publish(msgLaserScan);

      pub_rate.sleep();
    }
  }
  catch (exception& e)
  {
    ROS_FATAL("vrep_laser_driver_node node caught exception. Aborting. %s", e.what());
    ROS_BREAK();
  }




}




