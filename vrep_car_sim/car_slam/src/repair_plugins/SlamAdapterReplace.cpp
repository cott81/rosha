/*
 * SlamAdapterReplace.cpp
 *
 *  Created on: Jun 11, 2014
 *      Author: dominik
 */

#include <repair_plugins/SlamAdapterReplace.h>

using std::cout;
using std::endl;

namespace car_slam_repair_plugins
{

SlamAdapterReplace::SlamAdapterReplace()
: BaseRepair(), corrsepondingCompName("car_slam_node")
{
  this->pluginName = "SlamAdapterReplace";
  this->repairType = rosha_msgs::RepairAction::REPAIR_ACTION__VREP_SLAM_ADAPTER_REPLACE;
  //this->corrsepondingCompName = "car_localizer_node";

  std::string path;
  /*
  char* roshaRoot = NULL;

  roshaRoot = getenv("ROSHA_ROOT");

  if ( roshaRoot == NULL || strcmp(roshaRoot, "") == 0)
  {
    ROS_WARN("Environment variable ROSHA_ROOT is empty. Use three level above from ROS package path car slam.");
    std::string path = ros::package::getPath("car_slam");
    this->packagePath = path + "/../../../devel/lib/car_slam/";
  }
  else
  {
    std::string path(roshaRoot);
    this->packagePath = path + "/lib/car_slam/";
  }
  */

  //get path to this package ...
  path = ros::package::getPath("car_slam");
  path = path + "/src/repair_plugins/";
  this->pathedModelFilename = path + "Slam2SlamAdapterReplace.xml";


  int z;
  z = gethostname(hostname, sizeof hostname);
  this->fullName = "";
  std::stringstream ss;
  ss << this->hostname << "_" << "SLAM";
  ss >> this->fullName;

  //this->packagePath = "/home/dominik/work/rosha_ws/devel/lib/car_slam";

  this->diagDeactivatePup = nh->advertise<std_msgs::String>("/diagnostics_deactivate", 1000); //call Care to do some repair stuff
}

SlamAdapterReplace::~SlamAdapterReplace()
{
}

void SlamAdapterReplace::Initialize(void** data, int length)
{
  return;
}

void SlamAdapterReplace::Repair()
{
  ROS_INFO("%s: replace repair (car_slam_node -> car_slam_adapter)", this->pluginName.c_str());

  //send stop msg ... in case the comp is still alive ... no effect if not
  ROS_INFO("... stop car_slam_node");
  rosha_msgs::CareRepairControl stopMsg;
  stopMsg.robotId = this->ownId;
  stopMsg.repairActionToPerform = rosha_msgs::CareRepairControl::StopProcess;
  //stopMsg.compName = this->corrsepondingCompName; //Care manages func by given names ... here GPS
  stopMsg.compName = "SLAM";

  this->repairControlPup.publish(stopMsg);

  //time delay needed, because otherwise Care skips msgs
  std::this_thread::sleep_for(std::chrono::milliseconds(REPAIR_MSG_DELAY_MS));

  /*
  //send msg to Care with infos: target comp, new comp, parameter, workspace ... more?
  ROS_INFO("... replace car_slam_node with car_slam_adapter in the robot configuration model");
  rosha_msgs::CareRepairControl replaceMsg;
  replaceMsg.robotId = this->ownId;
  replaceMsg.compName = "SLAM";
  replaceMsg.compId = -1; //not known
  replaceMsg.repairActionToPerform = rosha_msgs::CareRepairControl::ReplaceProcess;
  replaceMsg.compToPlace.name = "SLAM_Adapter";
  ROS_INFO("SlamAdapterReplace::Repair: working directory: %s", this->packagePath.c_str());
  replaceMsg.compToPlace.workingDirectory = this->packagePath;
  replaceMsg.compToPlace.filename = "car_slam_adapter";
  replaceMsg.compToPlace.arguments = ""; //remote topics already hardcoded
  */


  //send msg to Care with infos: target comp, new comp, parameter, workspace ... more?
  ROS_INFO("... replace car_slam_node with car_slam_adapter in the robot configuration model");
  rosha_msgs::CareRepairControl replaceMsg;
  replaceMsg.robotId = this->ownId;
  replaceMsg.compName = "SLAM"; //this->targetCompName;
  replaceMsg.compId = -1; //not known
  replaceMsg.repairActionToPerform = rosha_msgs::CareRepairControl::RecomposeProcess;
  replaceMsg.structureToPlace.modelFilename = this->pathedModelFilename;



  this->repairControlPup.publish(replaceMsg);

  //time delay needed
  std::this_thread::sleep_for(std::chrono::milliseconds(REPAIR_MSG_DELAY_MS));

  //deactivates diagnosis for GPS
  ROS_INFO("... deactivates car_slam_node (SLAM) diagnosis");
  std_msgs::String deactivateDiagMsg_gps;
  deactivateDiagMsg_gps.data = this->fullName; //e.g. "iceland_SLAM"

  this->diagDeactivatePup.publish(deactivateDiagMsg_gps);

  std::this_thread::sleep_for(std::chrono::milliseconds(REPAIR_MSG_DELAY_MS));

  ROS_INFO("... (starts and) monitors the system");
  rosha_msgs::CareRepairControl startMonMsg;
  startMonMsg.robotId = this->ownId;
  startMonMsg.repairActionToPerform = rosha_msgs::CareRepairControl::StartNMonSys;
  startMonMsg.compName = "";

  this->repairControlPup.publish(startMonMsg);

  return;
}

} /* namespace car_slam_repair_plugins */
