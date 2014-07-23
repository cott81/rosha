/*
 * RemoteSlamProcessing.cpp
 *
 *  Created on: Jun 11, 2014
 *      Author: dominik
 */

#include <repair_plugins/RemoteSlamProcessing.h>

namespace vrep_slam_repair_plugins
{

RemoteSlamProcessing::RemoteSlamProcessing()
: corrsepondingCompName("vrep_slam_node"), REPAIR_MSG_DELAY(1)
{
  this->pluginName = "RemoteSlamProcessing";
  this->repairType = rosha_msgs::RepairAction::REPAIR_ACTION__VREP_SLAM_REMOTE_PROCESS;
  //this->corrsepondingCompName = "vrep_localizer_node";

  //get path to this package ...
  std::string path = ros::package::getPath("vrep_slam");
  path = path + "/src/repair_plugins/";
  this->pathedModelFilename = path + "RemoteSlamComp.xml";
}

RemoteSlamProcessing::~RemoteSlamProcessing()
{
  // TODO Auto-generated destructor stub
}

void RemoteSlamProcessing::Initialize(void** data, int length)
{
  return;
}

void RemoteSlamProcessing::Repair()
{
  ROS_INFO("%s: integrate slam for remote processing (vrep_slam_node)", this->pluginName.c_str());

  //send msg to Care with infos: target comp, new comp, parameter, workspace ... more?
  ROS_INFO("... integrate vrep_slam_node for remote processing in the robot configuration model");
  rosha_msgs::CareRepairControl remoteCompMsg;
  remoteCompMsg.robotId = this->ownId;
  remoteCompMsg.compName = "SLAM_REMOTE";
  remoteCompMsg.compId = -1; //not known
  remoteCompMsg.repairActionToPerform = rosha_msgs::CareRepairControl::IntegrateRemoteProcess;
  remoteCompMsg.structureToPlace.modelFilename = this->pathedModelFilename;
  remoteCompMsg.structureToPlace.failedRobotId = this->failedRobotId;

  this->repairControlPup.publish(remoteCompMsg);

  //time delay needed
  sleep(REPAIR_MSG_DELAY);

  /*
  //send start msg ... recovery manager that triggers this reoair is only active if the system should perform its task
  // ... not yet possible ... process dictionary is not yet updated with the new func name ... still the old!
  ROS_INFO("... (re)start vrep_slam_note remote");
  rosha_msgs::CareRepairControl startMsg;
  startMsg.robotId = this->ownId;
  startMsg.repairActionToPerform = rosha_msgs::CareRepairControl::StartProcess;
  startMsg.compName = "SLAM_REMOTE";

  this->repairControlPup.publish(startMsg);
  */

  ROS_INFO("... (starts and) monitors the system");
  rosha_msgs::CareRepairControl startMonMsg;
  startMonMsg.robotId = this->ownId;
  startMonMsg.repairActionToPerform = rosha_msgs::CareRepairControl::StartNMonSys;
  startMonMsg.compName = "";

  this->repairControlPup.publish(startMonMsg);

  return;
}

} /* namespace vrep_slam_repair_plugins */
