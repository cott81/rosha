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
#include <chrono>
#include <thread>

#include <ros/ros.h>
#include <rosha_msgs/CareRepairControl.h>
#include <rosha_msgs/RepairAction.h>

namespace gen_repair_plugins
{
  class BaseRepair
  {
    public:
      virtual void Initialize(void** data, int length) = 0;
      virtual ~BaseRepair(){}
      virtual void Repair() {
        std::cout << "BaseRepair: empty Repair" << std::endl;
      }
      virtual std::string GetName() {
        return "Error: Base Clase func call";
      }

      virtual void SetData(const rosha_msgs::RepairAction::ConstPtr& msg)
      {
        this->ownId = msg->robotId;
        this->targetCompId = msg->compId;
        this->targetCompName = msg->compName;
      }

      unsigned short repairType;


    protected:
      BaseRepair(int msgDelay=500) : REPAIR_MSG_DELAY_MS(msgDelay)
    {
        this->repairType = 100;
        this->nh = new ros::NodeHandle();
        this->repairControlPup = nh->advertise<rosha_msgs::CareRepairControl>("Care/RepairControl", 1000); //call Care to do some repair stuff
        this->targetCompId = -1;
        this->ownId = -1;
    }

      ros::NodeHandle* nh;
      ros::Publisher repairControlPup;
      int targetCompId;
      std::string targetCompName;
      int ownId;

      const int REPAIR_MSG_DELAY_MS;

  };
};


#endif /* BASEREPAIR_H_ */
