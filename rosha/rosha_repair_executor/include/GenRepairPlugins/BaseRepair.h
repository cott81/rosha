/*
 * BaseRepair.h
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
 *  Created on: Sep 6, 2012		12:31:53 PM
 *      Author: Dominik Kirchner
 */

#ifndef BASEREPAIR_H_
#define BASEREPAIR_H_

#include <iostream>

#include <ros/ros.h>
#include <rosha_msgs/CareRepairControl.h>

namespace gen_repair_plugins
{

// now defined in the msg
/*
enum repair_t {
  dummyMapperRedundancyRepair = 0,
  capFailureReport,
  restartRepair,
  ifaceRestart,
  testRepair = 100
} repairPlugins;
*/

//at first plain listing ... now in msg
/*
enum repairActionId_t {
  REPAIR_ACTION__RESTART = 0,
  REPAIR_ACTION__STOP,
  REPAIR_ACTION__START,
  REPAIR_ACTION__REPLACE
} repairActionId;
*/

  class BaseRepair
  {
    public:
      virtual void initialize() = 0;
      virtual ~BaseRepair(){}
      virtual void Repair() {
        std::cout << "BaseRepair: empty Repair" << std::endl;
      }
      virtual std::string GetName() {
        return "Error: Base Clase func call";
      }

      virtual void SetData(int compId, std::string compName, int ownId)
      {
        targetCompId = compId;
        targetCompName = compName;
        this->ownId = ownId;
      }

      unsigned short repairType;


    protected:
      BaseRepair()
    {
        this->repairType = 100;
        this->nh = new ros::NodeHandle();
        this->repairControlPup = nh->advertise<rosha_msgs::CareRepairControl>("Care/RepairControl", 1000); //call Care to do some repair stuff
    }

      ros::NodeHandle* nh;
      ros::Publisher repairControlPup;
      int targetCompId;
      std::string targetCompName;
      int ownId;

  };
};


#endif /* BASEREPAIR_H_ */
