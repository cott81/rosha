/*
 * SlamAdapterReplace.cpp
 *
 *  Created on: Jun 11, 2014
 *      Author: dominik
 */

#include <repair_plugins/SlamAdapterReplace.h>

namespace vrep_slam_repair_plugins
{

SlamAdapterReplace::SlamAdapterReplace()
: corrsepondingCompName("vrep_slam_node"), REPAIR_MSG_DELAY(1)
{
  this->pluginName = "SlamAdapterReplace";
  this->repairType = rosha_msgs::RepairAction::REPAIR_ACTION__VREP_SLAM_ADAPTER_REPLACE;
  //this->corrsepondingCompName = "vrep_localizer_node";
}

SlamAdapterReplace::~SlamAdapterReplace()
{
  // TODO Auto-generated destructor stub
}

void SlamAdapterReplace::Initialize(void** data, int length)
{
  //TODO replace hard coded path ... search in package path?
  this->packagePath = "/home/dominik/work/rosha_ws/devel/lib/vrep_slam";
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
  replaceMsg.compToPlace.workingDirectory = this->packagePath;
  replaceMsg.compToPlace.filename = "vrep_slam_adapter";
  replaceMsg.compToPlace.arguments = ""; //remote topics already hardcoded

  this->repairControlPup.publish(replaceMsg);

  //time delay needed
  sleep(REPAIR_MSG_DELAY);


  //send start msg ... recovery manager that triggers this reoair is only active if the system should perform its task
  // ... not yet possible ... process dictionary is not yet updated with the new func name ... still the old!
  ROS_INFO("... (re)start vrep_slam_adapter");
  rosha_msgs::CareRepairControl startMsg;
  startMsg.robotId = this->ownId;
  startMsg.repairActionToPerform = rosha_msgs::CareRepairControl::StartProcess;
  startMsg.compName = "SLAM_Adapter";

  this->repairControlPup.publish(startMsg);

  return;
}

} /* namespace vrep_slam_repair_plugins */
