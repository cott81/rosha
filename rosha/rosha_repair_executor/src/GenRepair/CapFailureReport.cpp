/*
 * CapFailureReport.cpp
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
 *  Created on: Sep 10, 2012		9:50:21 AM
 *      Author: Dominik Kirchner
 */

#include "../include/GenRepairPlugins/CapFailureReport.h"

//PLUGINLIB_DECLARE_CLASS(repair_executer_plugins, cap_report_plug, gen_repair_plugins::CapFailureReport, gen_repair_plugins::BaseRepair)

using namespace gen_repair_plugins;


CapFailureReport::CapFailureReport() {

  this->pluginName = "CapFailureReport";
}

CapFailureReport::~CapFailureReport() {
}

void  CapFailureReport::Initialize(void** data, int length)
{
  //use this for init some vars
  //side_length_ = side_length;
}

void CapFailureReport::Repair() {
  ROS_INFO("CapFailureReport::Repair: doing repair stuff ... report failure to alica to handle role changes ...");
  //information needed?
}

std::string CapFailureReport::GetName() {
  return this->pluginName;
}


