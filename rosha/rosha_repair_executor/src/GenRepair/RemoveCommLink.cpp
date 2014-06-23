/*
 * RemoveCommLink.cpp
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


#include "../include/GenRepairPlugins/RemoveCommLink.h"

using namespace gen_repair_plugins;
using namespace std;


RemoveCommLink::RemoveCommLink()
: CONF_FILE_NAME("relayMsgs.conf"), COMM_PROXY_COMP_NAME("MSLDDSProxy")
{
  this->pluginName = "RemoveCommLink";

  this->repairType = rosha_msgs::RepairAction::REPAIR_ACTION__REMOVE_COMM_LINK;

  this->stopRepairAction = rosha_msgs::CareRepairControl::StopProcess;
  this->startRepairAction = rosha_msgs::CareRepairControl::StartProcess;

  proxyPackagePath = ros::package::getPath(COMM_PROXY_COMP_NAME);
  this->pathedrelayConfFile = proxyPackagePath + "/" + CONF_FILE_NAME;
}

RemoveCommLink::~RemoveCommLink()
{
}

void  RemoveCommLink::Initialize(void** data, int length)
{
  //use this for init some vars
  //side_length_ = side_length;
}

void RemoveCommLink::SetData(const rosha_msgs::RepairAction::ConstPtr& msg)
{
  this->ownId = msg->robotId;
  this->targetCompId = msg->compId;

  this->targetCompName = msg->compName; //here topic name
  ParseTopics(this->targetCompName, this->topicNames);

  this->msgType = msg->msgType;
  ParseTopics(this->msgType, this->msgTypes);
}

void RemoveCommLink::Repair()
{

  if (this->topicNames.size() != this->msgTypes.size())
  {
    ROS_ERROR("Number of given topics (%d) and number of given types (%d) do not match!.", this->topicNames.size(), this->msgTypes.size());
  }
  ROS_INFO("doing repair stuff REMOVE COMM LINK.");

  // remove the topic in the file
  if (!RemoveTopicInFile(pathedrelayConfFile, this->topicNames))
  {
    ROS_WARN("... nothing found to remove in conf file. Nothing to do.");
  }

  //rebuild the code generaded udpProxy. call make
  string cmd = "make -C " + this->proxyPackagePath;
  ROS_INFO(" ... (re)make the udpProxy to integrate the new comm link. Call %s \n\n\n", cmd.c_str());
  int out = system(cmd.c_str());
  if (out != 0)
  {
    ROS_ERROR("RemoveCommLink: Reapair failed. Make failure with error code: %d", out);
  }
  cout << endl << endl << endl;

  //restart udpProxy. Care starts the proxy automatically and restarts it if it is killed
  string killCmd = "killall udpProxy";
  ROS_INFO("... restart/kill the udpProxy. Call %s", killCmd.c_str());
  out = system(killCmd.c_str());
  if (out != 0)
  {
    ROS_ERROR("RemoveCommLink: Restart( killall) of udpProxy failed with error code: %d. Assume that udpProxy was already started AND managed by Care", out);
  }

}

void RemoveCommLink::RemoveWhiteSpacesAtBegin(std::string& s)
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

bool RemoveCommLink::RemoveTopicInFile(const std::string& confFile, const std::vector<std::string>& topicsToRemove)
{
  bool removed = false;

  string line;
  ifstream relayMsgFileToCheck (confFile);
  string tempOutFileName = confFile+".tmp";
  ofstream tempOutFile (tempOutFileName);

  if (relayMsgFileToCheck.is_open() && tempOutFile.is_open())
  {
    while ( getline (relayMsgFileToCheck, line) )
    {
      //cout << line << endl;

      stringstream iss (line);

      //parse first static token: Topic
      string ss;
      getline(iss, ss, ':');

      //parse topic name parameter ... <topicName> ws  Msg:
      string topicNameInConf;
      getline(iss, topicNameInConf, ':');
      RemoveWhiteSpacesAtBegin(topicNameInConf);

      //check the line against all lines to remove
      bool found = false;

      for (int i=0; i<topicsToRemove.size(); i++)
      {
        if (!topicNameInConf.compare(0, topicsToRemove[i].size(), topicsToRemove[i]))
        {
          if (topicNameInConf.size() <= topicsToRemove[i].size())
          {
            //no match ... add to output file
          }
          else
          {
            //only match if the following char is a white space
            if (topicNameInConf[topicsToRemove[i].size()] == ' ' || topicNameInConf[topicsToRemove[i].size()] == '\t')
            {
              //cout << "MATCH !!!!" << endl;
              found = true;
              //remove this line ... do NOT add to output file!
              removed = true;
            }
          }
        }
      }

      if (found )
      {
        //found the line
        //remove this line ... do NOT add to output file!
        ROS_INFO("... removed line: %s", line.c_str());
      }
      else
      {
        // add not matching lines to output file
        tempOutFile << line << endl;
      }

    }
    relayMsgFileToCheck.close();
    tempOutFile.close();

    remove(confFile.c_str());
    rename(tempOutFileName.c_str() ,confFile.c_str());
  }
  else {
    printf("RemoveCommLink: Unable to open udp proxy conf file: %s, or temp output file: %s", confFile.c_str(), tempOutFileName.c_str());
    return true;
  }
  return removed;
}

std::string RemoveCommLink::GetName() {
  return this->pluginName;
}

int RemoveCommLink::ParseTopics(std::string& topics, std::vector<std::string>& parsedTopicNames)
{
  cout << topics << endl;

  //remove all whitespaces
  topics.erase(std::remove_if( topics.begin(), topics.end(),
       [](char c){ return (c =='\r' || c =='\t' || c == ' ' || c == '\n');}),
               topics.end() );

  cout << topics << endl;

  stringstream ss(topics);
  string topic;
  while(getline(ss, topic, ':'))
  {
    cout << topic << endl;
    parsedTopicNames.push_back(topic);
  }

  return parsedTopicNames.size();
}

