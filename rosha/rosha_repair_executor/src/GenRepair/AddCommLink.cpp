/*
 * AddCommLink.cpp
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


#include "../include/GenRepairPlugins/AddCommLink.h"

using namespace gen_repair_plugins;
using namespace std;


AddCommLink::AddCommLink()
: CONF_FILE_NAME("relayMsgs.conf"), COMM_PROXY_COMP_NAME("MSLDDSProxy")
{


  this->pluginName = "AddCommLink";

  this->repairType = rosha_msgs::RepairAction::REPAIR_ACTION__ADD_COMM_LINK;

  this->stopRepairAction = rosha_msgs::CareRepairControl::StopProcess;
  this->startRepairAction = rosha_msgs::CareRepairControl::StartProcess;

  proxyPackagePath = ros::package::getPath(COMM_PROXY_COMP_NAME);
  //string pathedrelayConfFile = path + "/relayMsgs.conf";
  this->pathedrelayConfFile = proxyPackagePath + "/" + CONF_FILE_NAME;
}

AddCommLink::~AddCommLink() {
}

void  AddCommLink::Initialize(void** data, int length)
{
  //use this for init some vars
  //side_length_ = side_length;
}

void AddCommLink::SetData(const rosha_msgs::RepairAction::ConstPtr& msg)
{
  this->ownId = msg->robotId;
  this->targetCompId = msg->compId;
  this->targetCompName = msg->compName; //here topic name
  this->msgType = msg->msgType;
}

void AddCommLink::Repair() {
  ROS_INFO("doing repair stuff ADD COMM LINK for topic: %s msgType: %s", this->targetCompName.c_str(), this->msgType.c_str());

  string topicName = this->targetCompName;
  string msgType = this->msgType;

  //add to relayMsgs.conf

  ROS_INFO(" ... add topic: %s (type: %s) to udp proxy conf: %s",
           topicName.c_str(), msgType.c_str(), pathedrelayConfFile.c_str() );

  stringstream ss;
  ss << "Topic: " << topicName << "\t\tMsg: " << msgType << "\t\tOpt:[] \n";
  string confTextLine = ss.str();

  // check if the topic is already present ... look for /testA
  if (CheckForIdenticalTopic(pathedrelayConfFile, topicName))
  {
    //already present ... nothing to do
    ROS_INFO("... topic: %s already in conf file. Nothing to do.", topicName.c_str());
    return;
  }

  //write to relayMsgs.conf
  ofstream relayMsgFile;
  relayMsgFile.open (pathedrelayConfFile, ios::out | ios::app);
  if (relayMsgFile.is_open()) {
    relayMsgFile << confTextLine;
    relayMsgFile.close();
  } else {
    ROS_ERROR("AddCommLink: Unable to open udp proxy conf file: %s", pathedrelayConfFile.c_str());
    return;
  }

  //rebuild the code generaded udpProxy. call make
  string cmd = "make -C " + this->proxyPackagePath;
  ROS_INFO(" ... (re)make the udpProxy to integrate the new comm link. Call %s \n\n\n", cmd.c_str());
  int out = system(cmd.c_str());
  if (out != 0) {
    ROS_ERROR("AddCommLink: Reapair failed. Make failure with error code: %d", out);
  }
  cout << endl << endl << endl;

  //restart udpProxy. Care starts the proxy automatically and restarts it if it is killed
  string killCmd = "killall udpProxy";
  ROS_INFO("... restart/kill the udpProxy. Call %s", killCmd.c_str());
  out = system(killCmd.c_str());
  if (out != 0) {
    ROS_ERROR("AddCommLink: Restart( killall) of udpProxy failed with error code: %d. Assume that udpProxy was already started AND managed by Care", out);
  }

}

void AddCommLink::RemoveWhiteSpacesAtBegin(std::string& s)
{
  for (int i=0; i<s.size(); i++) {
    if (s[i] == ' ' || s[i] == '\t')
    {
      s.erase(i, 1);
      i--;
    } else {
      break;
    }
  }

  return;
}

bool AddCommLink::CheckForIdenticalTopic(const std::string& confFile, const std::string& topic)
{
  bool alreayInFile = false;

  string line;
  ifstream relayMsgFileToCheck (confFile);
  if (relayMsgFileToCheck.is_open())
  {
    while ( getline (relayMsgFileToCheck, line) )
    {
      //cout << line << endl;

      //ignore comments
      if (line[0] == '#')
      {
        continue;
      }

      stringstream iss (line);

      //parse first static token: Topic
      string ss;
      getline(iss, ss, ':');
      RemoveWhiteSpacesAtBegin(ss);
      if (ss.compare("Topic") != 0)
      {
        //no valid line
        continue;
      }

      //parse topic name parameter ... <topicName> ws  Msg:
      string topicNameInConf;
      getline(iss, topicNameInConf, ':');
      RemoveWhiteSpacesAtBegin(topicNameInConf);

      if (!topicNameInConf.compare(0, topic.size(), topic))
      {
        if (topicNameInConf.size() <= topic.size())
        {
          //no match ...
        }
        else
        {
          //only match if the following char is a white space
          if (topicNameInConf[topic.size()] == ' ' || topicNameInConf[topic.size()] == '\t')
          {
            //cout << "MATCH !!!!" << endl;
            return true;
          }
        }
      }

    }
    relayMsgFileToCheck.close();
  }
  else {
    printf("AddCommLink: Unable to open udp proxy conf file: %s", confFile.c_str());
    return true;
  }
  return alreayInFile;
}

std::string AddCommLink::GetName() {
  return this->pluginName;
}

