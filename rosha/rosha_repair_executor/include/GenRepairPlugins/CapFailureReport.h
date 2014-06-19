/*
 * CapFailureReport.h
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
 *  Created on: Sep 10, 2012		9:49:03 AM
 *      Author: Dominik Kirchner
 */

#ifndef CAPFAILUREREPORT_H_
#define CAPFAILUREREPORT_H_


#include <iostream>

#include "ros/ros.h"

//#include <pluginlib/class_list_macros.h>
#include "GenRepairPlugins/BaseRepair.h"

namespace gen_repair_plugins
{
  class CapFailureReport : public BaseRepair
  {
    public:
    CapFailureReport();
      ~CapFailureReport();
      void Initialize(void** data, int length);
      void Repair();
      std::string GetName();

    private:
      std::string pluginName;
  };
}



#endif /* CAPFAILUREREPORT_H_ */
