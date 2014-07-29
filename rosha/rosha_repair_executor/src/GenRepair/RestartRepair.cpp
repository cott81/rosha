/*
 * RestartRepair.cpp
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


#include "../include/GenRepairPlugins/RestartRepair.h"

using namespace gen_repair_plugins;
using namespace std;


RestartRepair::RestartRepair() {

  this->pluginName = "RestartRepair";
  this->repairType = rosha_msgs::RepairAction::REPAIR_ACTION__RESTART;
}

RestartRepair::~RestartRepair() {
}

void  RestartRepair::Initialize(void** data, int length)
{
  //use this for init some vars
  //side_length_ = side_length;
}

void RestartRepair::Repair() {
  ROS_INFO("doing repair stuff RESTART ...");

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
  sleep(SLEEP_TIME);

  /*
  //send new message to restart the component
  rosha_msgs::CareRepairControl startMsg;
  startMsg.robotId = this->ownId;
  startMsg.repairActionToPerform = rosha_msgs::CareRepairControl::StartProcess;
  startMsg.compName = this->targetCompName;
  ROS_INFO("... send repair control msg to care: robotId: %d repairAction: %d compName: %s",
           startMsg.robotId, startMsg.repairActionToPerform, startMsg.compName.c_str());

  repairControlPup.publish(startMsg);
  */

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

std::string RestartRepair::GetName() {
  return this->pluginName;
}

