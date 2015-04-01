/*
 * StopRepair.cpp
 *
 * Copyright 2012 Carpe Noctem, Distributed Systems Group,
 * University of Kassel. All right reserved.
 *
 * The code is derived from the software contributed to Carpe Noctem by
 * the Carpe Noctem Team.
 *
 * The code is licensed under the Carpe Noctem Userfriendly BSD-Based
 * License (CNUBBL). Redistribution and use in source and binary forms,
 * with or without modification, are permitted provided that the
 * conditions of the CNUBBL are met.
 *
 * You should have received a copy of the CNUBBL along with this
 * software. The license is also available on our website:
 * http://carpenoctem.das-lab.net/license.txt
 *  Created on: Sep 7, 2012		2:38:30 PM
 *      Author: Dominik Kirchner
 */


#include "../include/GenRepairPlugins/StopRepair.h"

using namespace gen_repair_plugins;
using namespace std;


StopRepair::StopRepair() {

  this->pluginName = "StopRepair";
  this->repairType = rosha_msgs::RepairAction::REPAIR_ACTION__STOP;

  this->diagDeactivatePup = nh->advertise<std_msgs::String>("/diagnostics_deactivate", 1000); //call Care to do some repair stuff

  this->robotname =  supplementary::SystemConfig::getHostname();

  //get path to this package ...
  std::string path = ros::package::getPath("rosha_repair_executor");
  path = path + "/src/GenRepair/";
  this->pathedModelFilename = path + "Empty_RecomposeModel.xml";
}

StopRepair::~StopRepair() {
}

void  StopRepair::Initialize(void** data, int length)
{
  //use this for init some vars
  //side_length_ = side_length;

  std::stringstream ss;
  ss << this->robotname << "_" << this->targetCompName;
  ss >> this->fullName;
}

void StopRepair::Repair() {
  ROS_INFO("doing repair stuff STOP ...");

  // universal repair action (same for all components anf failures types) / indirect (command Care to do the restart)

  //send msg ...
  rosha_msgs::CareRepairControl stopMsg;
  stopMsg.robotId = this->ownId;
  stopMsg.repairActionToPerform = rosha_msgs::CareRepairControl::StopProcess;
  stopMsg.compName = this->targetCompName;
  ROS_INFO("... send repair control msg to care: robotId: %d repairAction: %d compName: %s",
           stopMsg.robotId, stopMsg.repairActionToPerform, stopMsg.compName.c_str());

  repairControlPup.publish(stopMsg);

  //wait for some time ...
  std::this_thread::sleep_for(std::chrono::milliseconds(REPAIR_MSG_DELAY_MS));


  //HACK: remove func from robot tree by replacing it with an executable on in the composition file!!!
  //send msg to Care with infos: target comp, new comp, parameter, workspace ... more?
  ROS_INFO("... replace (remove) node with empty composition in the robot configuration model");
  rosha_msgs::CareRepairControl replaceMsg;
  replaceMsg.robotId = this->ownId;
  replaceMsg.compName = this->targetCompName;
  replaceMsg.compId = -1; //not known
  replaceMsg.repairActionToPerform = rosha_msgs::CareRepairControl::RecomposeProcess;
  replaceMsg.structureToPlace.modelFilename = this->pathedModelFilename;

  repairControlPup.publish(replaceMsg);
  //wait for some time ...
  std::this_thread::sleep_for(std::chrono::milliseconds(REPAIR_MSG_DELAY_MS));

  std::stringstream ss;
  ss << this->robotname << "_" << this->targetCompName;
  ss >> this->fullName;

  //deactivates diagnosis for GPS
  ROS_INFO("... deactivates %s diagnosis", this->fullName.c_str());
  std_msgs::String deactivateDiagMsg_gps;
  deactivateDiagMsg_gps.data = this->fullName; //e.g. "iceland_GPS"

  this->diagDeactivatePup.publish(deactivateDiagMsg_gps);

  //wait for some time ...
  std::this_thread::sleep_for(std::chrono::milliseconds(REPAIR_MSG_DELAY_MS));

  ROS_INFO("... (starts and) monitors the system");
  rosha_msgs::CareRepairControl startMonMsg;
  startMonMsg.robotId = this->ownId;
  startMonMsg.repairActionToPerform = rosha_msgs::CareRepairControl::StartNMonSys;
  startMonMsg.compName = "";
  ROS_INFO("... send repair control msg to care: robotId: %d repairAction: %d compName: %s",
           startMonMsg.robotId, startMonMsg.repairActionToPerform, startMonMsg.compName.c_str());

  this->repairControlPup.publish(startMonMsg);


  //feedback to the recovery controller if successfull or not ... cant say here! -> no feedback
  // ... perhaps feedback if repair sequence was finshed
}

std::string StopRepair::GetName() {
  return this->pluginName;
}

