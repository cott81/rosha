/*
 * CompRedundancyRepair.h
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
 *  Created on: Sep 7, 2012		11:13:36 AM
 *      Author: Dominik Kirchner
 */

#ifndef COMPREDUNDANCYREPAIR_H_
#define COMPREDUNDANCYREPAIR_H_

#include "ros/ros.h"

#include <GenRepairPlugins/BaseRepair.h>
#include <rosha_msgs/RepairAction.h>


namespace vrep_localization_repair_plugins
{
  class CompRedundancyRepair : public gen_repair_plugins::BaseRepair
  {
    public:
      CompRedundancyRepair();
      ~CompRedundancyRepair();
      void Initialize(void** data, int length);
      void Repair();
      std::string GetName();

    private:
      std::string pluginName;
      void DoRedundancyRepair();
  };
}



#endif /* COMPREDUNDANCYREPAIR_H_ */
