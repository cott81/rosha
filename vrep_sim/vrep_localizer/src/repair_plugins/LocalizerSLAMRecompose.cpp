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

  //get path to this package ...
  std::string path = ros::package::getPath("vrep_localizer");
  path = path + "/src/repair_plugins/";
  this->pathedModelFilename = path + "LocalizerSLAMRecomposeModel.xml";

  this->diagDeactivatePup = nh->advertise<std_msgs::String>("/diagnostics_deactivate", 1000); //call Care to do some repair stuff
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
  sleep(REPAIR_MSG_DELAY);

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
  sleep(REPAIR_MSG_DELAY);

  //send start msg ... recovery manager that triggers this reoair is only active if the system should perform its task
  // start all funcs in the model ... maunal by name
  //TODO: replace the hard coded names ... read this from the model file

  //do not need this, because we simply start the complete system
  /*
  ROS_INFO("... start vrep_laser_driver_node");
  rosha_msgs::CareRepairControl startMsg_laser;
  startMsg_laser.robotId = this->ownId;
  startMsg_laser.repairActionToPerform = rosha_msgs::CareRepairControl::StartProcess;
  startMsg_laser.compName = "LaserDriver";

  this->repairControlPup.publish(startMsg_laser);

  sleep(REPAIR_MSG_DELAY);

  ROS_INFO("... start vrep_slam_node");
  rosha_msgs::CareRepairControl startMsg_slam;
  startMsg_slam.robotId = this->ownId;
  startMsg_slam.repairActionToPerform = rosha_msgs::CareRepairControl::StartProcess;
  startMsg_slam.compName = "SLAM";

  this->repairControlPup.publish(startMsg_slam);
  */

  //deactivates diagnosis for GPS
  ROS_INFO("... deactivates vrep_localizer_node (GPS) diagnosis");
  std_msgs::String deactivateDiagMsg_gps;
  deactivateDiagMsg_gps.data = "iceland_GPS";

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

} /* namespace vrep_localization_repair_plugins */
