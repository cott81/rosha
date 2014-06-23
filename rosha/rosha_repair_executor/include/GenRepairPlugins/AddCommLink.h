/*
 * AddCommLink.h
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

#ifndef ADDCOMMLINK_H_
#define ADDCOMMLINK_H_

#include <iostream>
#include <fstream>
#include <sstream>

#include <unistd.h>

#include "ros/ros.h"
#include <ros/package.h>

#include <rosha_msgs/CareRepairControl.h>
#include <rosha_msgs/RepairAction.h>

//#include <pluginlib/class_list_macros.h>
#include "GenRepairPlugins/BaseRepair.h"

namespace gen_repair_plugins
{
  class AddCommLink : public BaseRepair
  {
    public:
      AddCommLink();
      ~AddCommLink();
      void Initialize(void** data, int length);
      void Repair();
      std::string GetName();
      void SetData(const rosha_msgs::RepairAction::ConstPtr& msg);

      //const char* CONF_FILE_NAME;
      const std::string COMM_PROXY_COMP_NAME;
      const std::string CONF_FILE_NAME;
      std::string proxyPackagePath;
      std::string pathedrelayConfFile;

    private:
      std::string pluginName;
      int stopRepairAction;
      int startRepairAction;
      static const int SLEEP_TIME = 3;
      std::string msgType;
      std::vector<std::string> topicNames;
      std::vector<std::string> msgTypes;


      void RemoveWhiteSpacesAtBegin(std::string& s);
      bool CheckForIdenticalTopic(const std::string& confFile, const std::string& topic);
      int ParseTopics(std::string& topics, std::vector<std::string>& parsedTopicNames);
  };
}


#endif /* ADDCOMMLINK_H_ */
