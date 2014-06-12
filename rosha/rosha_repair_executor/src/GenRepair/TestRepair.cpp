/*
 * TestRepair.cpp
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
 *  Created on: Sep 6, 2012		12:36:10 PM
 *      Author: Dominik Kirchner
 */

#include <GenRepairPlugins/TestRepair.h>

//PLUGINLIB_DECLARE_CLASS(repair_executer_plugins, test_plug, gen_repair_plugins::TestRepair, gen_repair_plugins::BaseRepair)

using namespace gen_repair_plugins;


TestRepair::TestRepair() {

  this->pluginName = "TestRepair";
  //just for testing ...
  this->repairType = 81;
}

TestRepair::~TestRepair() {
}

void  TestRepair::initialize()
{
  //use this for init some vars
  //side_length_ = side_length;
}

void TestRepair::Repair() {
  ROS_INFO("doing repair stuff ...");
  ROS_INFO("dummy TEST repair action JUST FOR TESTING");
  //std::cout << "doing repair stuff ..." << std::endl;
}

std::string TestRepair::GetName() {
  return this->pluginName;
}
