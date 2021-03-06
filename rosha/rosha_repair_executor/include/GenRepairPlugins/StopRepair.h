/*
 * StopRepair.h
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
 *  Created on: Sep 7, 2012		2:36:45 PM
 *      Author: Dominik Kirchner
 */

#ifndef STOPREPAIR_H_
#define STOPREPAIR_H_

#include <iostream>

#include "ros/ros.h"
#include <ros/package.h>
#include <rosha_msgs/CareRepairControl.h>
#include <rosha_msgs/RepairAction.h>

#include <std_msgs/String.h>

#include <SystemConfig.h>

//#include <pluginlib/class_list_macros.h>
#include "GenRepairPlugins/BaseRepair.h"

namespace gen_repair_plugins
{
  class StopRepair : public BaseRepair
  {
    public:
      StopRepair();
      ~StopRepair();
      void Initialize(void** data, int length);
      void Repair();
      std::string GetName();

    private:
      std::string pluginName;
      std::string robotname;
      std::string fullName;
      std::string pathedModelFilename;

      ros::Publisher diagDeactivatePup;
  };
}


#endif /* STOPREPAIR_H_ */
