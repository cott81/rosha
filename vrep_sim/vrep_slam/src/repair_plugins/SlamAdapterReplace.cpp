/*
 * SlamAdapterReplace.cpp
 *
 *  Created on: Jun 11, 2014
 *      Author: dominik
 */

#include <repair_plugins/SlamAdapterReplace.h>

using std::cout;
using std::endl;

namespace vrep_slam_repair_plugins
{

SlamAdapterReplace::SlamAdapterReplace()
: corrsepondingCompName("vrep_slam_node"), REPAIR_MSG_DELAY(1)
{
  this->pluginName = "SlamAdapterReplace";
  this->repairType = rosha_msgs::RepairAction::REPAIR_ACTION__VREP_SLAM_ADAPTER_REPLACE;
  //this->corrsepondingCompName = "vrep_localizer_node";

  std::string path;
  char* roshaRoot = NULL;

  roshaRoot = getenv("ROSHA_ROOT");

  if ( roshaRoot == NULL || strcmp(roshaRoot, "") == 0)
  {
    ROS_WARN("Environment variable ROSHA_ROOT is empty. Use three level above from ROS package path vrep slam.");
    std::string path = ros::package::getPath("vrep_slam");
    this->packagePath = path + "/../../../devel/lib/vrep_slam/";
  }
  else
  {
    std::string path(roshaRoot);
    this->packagePath = path + "/lib/vrep_slam/";
  }

  int z;
  z = gethostname(hostname, sizeof hostname);
  this->fullName = "";
  std::stringstream ss;
  ss << this->hostname << "_" << "SLAM";
  ss >> this->fullName;

  //this->packagePath = "/home/dominik/work/rosha_ws/devel/lib/vrep_slam";

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
  ROS_INFO("%s: replace repair (vrep_slam_node -> vrep_slam_adapter)", this->pluginName.c_str());

  //send stop msg ... in case the comp is still alive ... no effect if not
  ROS_INFO("... stop vrep_slam_node");
  rosha_msgs::CareRepairControl stopMsg;
  stopMsg.robotId = this->ownId;
  stopMsg.repairActionToPerform = rosha_msgs::CareRepairControl::StopProcess;
  //stopMsg.compName = this->corrsepondingCompName; //Care manages func by given names ... here GPS
  stopMsg.compName = "SLAM";

  this->repairControlPup.publish(stopMsg);

  //time delay needed, because otherwise Care skips msgs
  sleep(REPAIR_MSG_DELAY);

  //send msg to Care with infos: target comp, new comp, parameter, workspace ... more?
  ROS_INFO("... replace vrep_slam_node with vrep_slam_adapter in the robot configuration model");
  rosha_msgs::CareRepairControl replaceMsg;
  replaceMsg.robotId = this->ownId;
  replaceMsg.compName = "SLAM";
  replaceMsg.compId = -1; //not known
  replaceMsg.repairActionToPerform = rosha_msgs::CareRepairControl::ReplaceProcess;
  replaceMsg.compToPlace.name = "SLAM_Adapter";
  ROS_INFO("SlamAdapterReplace::Repair: working directory: %s", this->packagePath.c_str());
  replaceMsg.compToPlace.workingDirectory = this->packagePath;
  replaceMsg.compToPlace.filename = "vrep_slam_adapter";
  replaceMsg.compToPlace.arguments = ""; //remote topics already hardcoded

  this->repairControlPup.publish(replaceMsg);

  //time delay needed
  sleep(REPAIR_MSG_DELAY);

  /*
  //send start msg ... recovery manager that triggers this reoair is only active if the system should perform its task
  // ... not yet possible ... process dictionary is not yet updated with the new func name ... still the old!
  ROS_INFO("... (re)start vrep_slam_adapter");
  rosha_msgs::CareRepairControl startMsg;
  startMsg.robotId = this->ownId;
  startMsg.repairActionToPerform = rosha_msgs::CareRepairControl::StartProcess;
  startMsg.compName = "SLAM_Adapter";

  this->repairControlPup.publish(startMsg);
  */

  //deactivates diagnosis for GPS
  ROS_INFO("... deactivates vrep_slam_node (SLAM) diagnosis");
  std_msgs::String deactivateDiagMsg_gps;
  deactivateDiagMsg_gps.data = this->fullName; //e.g. "iceland_SLAM"

  this->diagDeactivatePup.publish(deactivateDiagMsg_gps);

  sleep(REPAIR_MSG_DELAY);

  ROS_INFO("... (starts and) monitors the system");
  rosha_msgs::CareRepairControl startMonMsg;
  startMonMsg.robotId = this->ownId;
  startMonMsg.repairActionToPerform = rosha_msgs::CareRepairControl::StartNMonSys;
  startMonMsg.compName = "";

  this->repairControlPup.publish(startMonMsg);

  return;
}

} /* namespace vrep_slam_repair_plugins */
