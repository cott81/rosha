/*
 * GPS_SLAM_Recompose.cpp
 *
 *  Created on: Jun 12, 2014
 *      Author: dominik
 */

#include <repair_plugins/GPS_SLAM_Recompose.h>

namespace car_localization_repair_plugins
{

GPS_SLAM_Recompose::GPS_SLAM_Recompose()
: BaseRepair(), corrsepondingCompName("vrep_localizer_node")
{
  this->pluginName = "GPS_SLAM_Recompose";
  this->repairType = rosha_msgs::RepairAction::REPAIR_ACTION__CAR_LOC_RECOMPOSE;

  //get path to this package ...
  std::string path = ros::package::getPath("car_localizer");
  path = path + "/src/repair_plugins/";
  this->pathedModelFilename = path + "GPS_SLAM_RecomposeModel.xml";

  this->diagDeactivatePup = nh->advertise<std_msgs::String>("/diagnostics_deactivate", 1000); //call Care to do some repair stuff

  int z;
  //z = gethostname(hostname, sizeof hostname);
  this->robotname =  supplementary::SystemConfig::getHostname();
  //char to string as stream operation
  this->fullName = "";
  std::stringstream ss;
  ss << this->robotname << "_" << "GPS";
  ss >> this->fullName;
}

GPS_SLAM_Recompose::~GPS_SLAM_Recompose()
{
}

void GPS_SLAM_Recompose::Initialize(void** data, int length)
{
  return;
}

void GPS_SLAM_Recompose::Repair()
{
  ROS_INFO("%s: recompose repair (vrep_localizer_node -> <slam>  (vrep_laser_scanner, vrep_slam)", this->pluginName.c_str());

  //send stop msg ... in case the comp is still alive ... no effect if not
  ROS_INFO("... stop car_localizer_node");
  rosha_msgs::CareRepairControl stopMsg;
  stopMsg.robotId = this->ownId;
  stopMsg.repairActionToPerform = rosha_msgs::CareRepairControl::StopProcess;
  //stopMsg.compName = this->corrsepondingCompName; //Care manages func by given names ... here GPS
  stopMsg.compName = this->targetCompName;

  this->repairControlPup.publish(stopMsg);

  //time delay needed, because otherwise Care skips msgs
  std::this_thread::sleep_for(std::chrono::milliseconds(REPAIR_MSG_DELAY_MS));

  //send msg to Care with infos: target comp, new comp, parameter, workspace ... more?
  ROS_INFO("... replace car_localizer_node with car_slam composition in the robot configuration model");
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
