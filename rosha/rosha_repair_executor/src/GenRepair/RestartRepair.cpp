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

//PLUGINLIB_DECLARE_CLASS(repair_executer_plugins, restart_plug, gen_repair_plugins::RestartRepair, gen_repair_plugins::BaseRepair)

using namespace gen_repair_plugins;


RestartRepair::RestartRepair() {

  this->pluginName = "RestartRepair";
}

RestartRepair::~RestartRepair() {
}

void  RestartRepair::initialize()
{
  //use this for init some vars
  //side_length_ = side_length;
}

void RestartRepair::Repair() {
  ROS_INFO("doing repair stuff ...");
  //std::cout << "doing repair stuff ..." << std::endl;
}

std::string RestartRepair::GetName() {
  return this->pluginName;
}

