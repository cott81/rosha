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
: BaseRepair(100),
  PLUGIN_NAME("RemoteSlamProcessing"),
  failedRobotId(-1)
{
  this->repairType = rosha_msgs::RepairAction::REPAIR_ACTION__VREP_SLAM_REMOTE_PROCESS;

  std::string path = ros::package::getPath("vrep_slam");
  path = path + "/src/repair_plugins/";
  this->pathedModelFilename = path + "RemoteSlamComp.xml";
}

RemoteSlamProcessing::~RemoteSlamProcessing()
{
}

void RemoteSlamProcessing::Initialize(void** data, int length)
{
  return;
}

void RemoteSlamProcessing::Repair()
{
  ROS_INFO("%s: integrate slam for remote processing (vrep_slam_node)", this->PLUGIN_NAME.c_str());

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
  std::this_thread::sleep_for(std::chrono::milliseconds(REPAIR_MSG_DELAY_MS));

  ROS_INFO("... (starts and) monitors the system");
  rosha_msgs::CareRepairControl startMonMsg;
  startMonMsg.robotId = this->ownId;
  startMonMsg.repairActionToPerform = rosha_msgs::CareRepairControl::StartNMonSys;
  startMonMsg.compName = "";

  this->repairControlPup.publish(startMonMsg);

  return;
}

} /* namespace vrep_slam_repair_plugins */
