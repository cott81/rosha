/*
 * CompRedundancyRepair.cpp
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
 *  Created on: Sep 7, 2012		11:13:59 AM
 *      Author: Dominik Kirchner
 */



#include <repair_plugins/CompRedundancyRepair.h>

using namespace vrep_localization_repair_plugins;


CompRedundancyRepair::CompRedundancyRepair() {

  this->pluginName = "DummyMapperRedundancyRepair";
  //this->repairType = rosha_msgs::RepairAction::REPAIR_ACTION__VREP_LOC_REPLACE;
  this->repairType = 101;
}

CompRedundancyRepair::~CompRedundancyRepair() {

}

void  CompRedundancyRepair::initialize()
{
  //inti vars
}

void CompRedundancyRepair::Repair() {
  ROS_INFO("doing repair stuff in DummyMapper Comp...");
}

std::string CompRedundancyRepair::GetName() {
  return this->pluginName;
}

void CompRedundancyRepair::DoRedundancyRepair() {
  ROS_INFO("... redundancy work ... to fill");
  // send stop cmd to care ...
  // send start cmd , or start it self
  //update care ... ???
}
