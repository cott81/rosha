/*
 * LocalizerRedundantReplace.cpp
 *
 *  Created on: Jun 11, 2014
 *      Author: dominik
 */

#include <repair_plugins/LocalizerRedundantReplace.h>

namespace vrep_localization_repair_plugins
{

LocalizerRedundantReplace::LocalizerRedundantReplace()
: corrsepondingCompName("vrep_localizer_node"), REPAIR_MSG_DELAY(1)
{
  this->pluginName = "LocalizerRedundantReplace";
  this->repairType = rosha_msgs::RepairAction::REPAIR_ACTION__VREP_LOC_REPLACE;
  //this->corrsepondingCompName = "vrep_localizer_node";
}

LocalizerRedundantReplace::~LocalizerRedundantReplace()
{
  // TODO Auto-generated destructor stub
}

void LocalizerRedundantReplace::Initialize(void** data, int length)
{
  return;
}

void LocalizerRedundantReplace::Repair()
{
  ROS_INFO("%s: replace repair (vrep_localizer_node -> vrep_localizer_redundant)", this->pluginName.c_str());

  //send stop msg ... in case the comp is still alive ... no effect if not
  ROS_INFO("... stop vrep_localizer_node");
  rosha_msgs::CareRepairControl stopMsg;
  stopMsg.robotId = this->ownId;
  stopMsg.repairActionToPerform = rosha_msgs::CareRepairControl::StopProcess;
  //stopMsg.compName = this->corrsepondingCompName; //Care manages func by given names ... here GPS
  stopMsg.compName = "GPS";

  this->repairControlPup.publish(stopMsg);

  //time delay needed, because otherwise Care skips msgs
  sleep(REPAIR_MSG_DELAY);

  //send msg to Care with infos: target comp, new comp, parameter, workspace ... more?
  ROS_INFO("... replace vrep_localizer_node with vrep_localizer_redundant in the robot configuration model");
  rosha_msgs::CareRepairControl replaceMsg;
  replaceMsg.robotId = this->ownId;
  replaceMsg.compName = "GPS";
  replaceMsg.compId = -1; //not known
  replaceMsg.repairActionToPerform = rosha_msgs::CareRepairControl::ReplaceProcess;
  replaceMsg.compToPlace.name = "GPS2";
  replaceMsg.compToPlace.workingDirectory = "/home/dominik/work/rosha_ws/devel/lib/vrep_localizer";
  replaceMsg.compToPlace.filename = "vrep_localizer_redundant";
  replaceMsg.compToPlace.arguments = "";

  this->repairControlPup.publish(replaceMsg);

  //time delay needed
  sleep(REPAIR_MSG_DELAY);


  //send start msg ... recovery manager that triggers this reoair is only active if the system should perform its task
  // ... not yet possible ... process dictionary is not yet updated with the new func name ... still the old!
  ROS_INFO("... (re)start vrep_localizer_node");
  rosha_msgs::CareRepairControl startMsg;
  startMsg.robotId = this->ownId;
  startMsg.repairActionToPerform = rosha_msgs::CareRepairControl::StartProcess;
  startMsg.compName = "GPS2";

  this->repairControlPup.publish(startMsg);

  return;
}

} /* namespace vrep_localization_repair_plugins */
