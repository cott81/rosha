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
: corrsepondingCompName("vrep_localizer_node"), REPAIR_MSG_DELAY(1)
{
  this->pluginName = "LocalizerSLAMRecompose";
  this->repairType = rosha_msgs::RepairAction::REPAIR_ACTION__VREP_LOC_RECOMPOSE;
}

LocalizerSLAMRecompose::~LocalizerSLAMRecompose()
{
  // TODO Auto-generated destructor stub
}

void LocalizerSLAMRecompose::Initialize(void** data, int length)
{
  return;
}

void LocalizerSLAMRecompose::Repair()
{
  ROS_INFO("%s: recompose repair (vrep_localizer_node -> <slam>  (vrep_laser_scanner, vrep_slam)", this->pluginName.c_str());

  //stop localizer component

  //send recompose msg ... what infos are needed ... just the filename

  //start all (both) components in the composition ... manual ...


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
  replaceMsg.repairActionToPerform = rosha_msgs::CareRepairControl::RecomposeProcess;

  //get path to this package ...
  std::string path = ros::package::getPath("vrep_localizer");
  path = path + "/src/repair_plugins/";
  std::string pathedModelFilename = path + "LocalizerSLAMRecomposeModel.xml";
  replaceMsg.structureToPlace.modelFilename = pathedModelFilename;


  this->repairControlPup.publish(replaceMsg);

  //time delay needed
  sleep(REPAIR_MSG_DELAY);


  //send start msg ... recovery manager that triggers this reoair is only active if the system should perform its task
  // start all funcs in the model ... maunal by name

  ROS_INFO("... start vrep_laser_driver_node");
  rosha_msgs::CareRepairControl startMsg_laser;
  startMsg_laser.robotId = this->ownId;
  startMsg_laser.repairActionToPerform = rosha_msgs::CareRepairControl::StartProcess;
  startMsg_laser.compName = "LaserDriver";

  this->repairControlPup.publish(startMsg_laser);

  ROS_INFO("... start vrep_slam_node");
  rosha_msgs::CareRepairControl startMsg_slam;
  startMsg_slam.robotId = this->ownId;
  startMsg_slam.repairActionToPerform = rosha_msgs::CareRepairControl::StartProcess;
  startMsg_slam.compName = "SLAM";

  this->repairControlPup.publish(startMsg_slam);


  return;
}

} /* namespace vrep_localization_repair_plugins */
