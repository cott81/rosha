/*
 * LocalizerSLAMRecompose.cpp
 *
 *  Created on: Jun 12, 2014
 *      Author: dominik
 */

#include <repair_plugins/LocalizerSLAMRecompose.h>

namespace vrep_localization_repair_plugins
{

LocalizerSLAMRecompose::LocalizerSLAMRecompose()
: BaseRepair(), corrsepondingCompName("vrep_localizer_node")
{
  this->pluginName = "LocalizerSLAMRecompose";
  this->repairType = rosha_msgs::RepairAction::REPAIR_ACTION__VREP_LOC_RECOMPOSE;

  //get path to this package ...
  std::string path = ros::package::getPath("vrep_localizer");
  path = path + "/src/repair_plugins/";
  this->pathedModelFilename = path + "LocalizerSLAMRecomposeModel.xml";

  this->diagDeactivatePup = nh->advertise<std_msgs::String>("/diagnostics_deactivate", 1000); //call Care to do some repair stuff

  int z;
  z = gethostname(hostname, sizeof hostname);
  this->fullName = "";
  std::stringstream ss;
  ss << this->hostname << "_" << "GPS";
  ss >> this->fullName;
}

LocalizerSLAMRecompose::~LocalizerSLAMRecompose()
{
}

void LocalizerSLAMRecompose::Initialize(void** data, int length)
{
  return;
}

void LocalizerSLAMRecompose::Repair()
{
  ROS_INFO("%s: recompose repair (vrep_localizer_node -> <slam>  (vrep_laser_scanner, vrep_slam)", this->pluginName.c_str());

  //send stop msg ... in case the comp is still alive ... no effect if not
  ROS_INFO("... stop vrep_localizer_node");
  rosha_msgs::CareRepairControl stopMsg;
  stopMsg.robotId = this->ownId;
  stopMsg.repairActionToPerform = rosha_msgs::CareRepairControl::StopProcess;
  //stopMsg.compName = this->corrsepondingCompName; //Care manages func by given names ... here GPS
  stopMsg.compName = this->targetCompName;

  this->repairControlPup.publish(stopMsg);

  //time delay needed, because otherwise Care skips msgs
  std::this_thread::sleep_for(std::chrono::milliseconds(REPAIR_MSG_DELAY_MS));

  //send msg to Care with infos: target comp, new comp, parameter, workspace ... more?
  ROS_INFO("... replace vrep_localizer_node with vrep_localizer_redundant in the robot configuration model");
  rosha_msgs::CareRepairControl replaceMsg;
  replaceMsg.robotId = this->ownId;
  replaceMsg.compName = this->targetCompName;
  replaceMsg.compId = -1; //not known
  replaceMsg.repairActionToPerform = rosha_msgs::CareRepairControl::RecomposeProcess;
  replaceMsg.structureToPlace.modelFilename = this->pathedModelFilename;

  this->repairControlPup.publish(replaceMsg);

  //time delay needed
  std::this_thread::sleep_for(std::chrono::milliseconds(REPAIR_MSG_DELAY_MS));

  //deactivates diagnosis for GPS
  ROS_INFO("... deactivates vrep_localizer_node (GPS) diagnosis");
  std_msgs::String deactivateDiagMsg_gps;
  deactivateDiagMsg_gps.data = this->fullName; //e.g. "iceland_GPS"

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

} /* namespace vrep_localization_repair_plugins */
