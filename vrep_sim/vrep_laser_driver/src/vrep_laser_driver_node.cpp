/*
 * vrep_localizer_node.cpp
 *
 *  Created on: May 22, 2014
 *      Author: dominik
 */

#include <vrep_laser_driver/VrepLaserDriver.h>

#include "ros/ros.h"
#include "vrep_msgs/LaserScanData.h"
#include "std_msgs/Float64.h"
#include "error_seeder/ErrorSeederLib.h"

#include "SystemConfig.h"

#include <exception>

using namespace std;
using namespace vrep_laser_driver;

VrepLaserDriver* vrepLaserDriver;

ros::Publisher* laserScanDataPublisher;

int robotId = 0; //robotId

/*
void SimLaserDataCmdCallback(const vrep_msgs::DriveCmd::ConstPtr& msg)
{
  cout << "get laser data  input" << endl;
}
*/

int main (int argc, char** argv)
{
  bool robotIdByArg = false;
  bool useRobotIdInTopic = false;
  string help = "Vrep Laser Driver\n"
      "Synobsis: vrep_laser_driver_node OPTIONS\n"
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

  cout << "start v_rep_laser_driver" << endl;
  vrepLaserDriver = new VrepLaserDriver(); //currently empty ...

  ros::init(argc, argv, "vrep_laser_driver_node");
  ros::NodeHandle n;

  if (!robotIdByArg)
  {
    robotId = supplementary::SystemConfig::GetOwnRobotID();
  }
  ROS_INFO("own robot Id: %d\n", robotId);

  error_seeder::ErrorSeederLib esl(ownId);

  // build topic name
  string pubTopic;
  stringstream ss;
  if (useRobotIdInTopic) {
    ss << "/vrep/MagicCube" << robotId << "/LaserScanData";
    ss >> pubTopic;
  }
  else
  {
    pubTopic = "/vrep/MagicCube/LaserScanData";
    // no interface to vrep yet, just provide some dummy data (see main loop)
  }


  ros::Publisher pub = n.advertise<vrep_msgs::LaserScanData>(pubTopic, 1000);
  laserScanDataPublisher = &pub;

  //ros::Subscriber motionCmdSub = n.subscribe("/vrep/MagicCubeXY/LaserScanDataFromSimulator", 1000, SimLaserDataCmdCallback);

  try
  {
    ros::Rate pub_rate(30.0);
    while (ros::ok())
    {
      ros::spinOnce();

      //TODO: heart beat msg
      cout << "loop"<<endl;

      //instead forwarding/ processing/ noising/ data from the simulator
      vrep_msgs::LaserScanData msgLaserScan;
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




