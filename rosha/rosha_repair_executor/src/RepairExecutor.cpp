/*
 * RepairExecuter.cpp
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
 *  Created on: Sep 5, 2012             2:38:30 PM
 *      Author: Dominik Kirchner
 */


#include "../include/repair_executor/RepairExecutor.h"


//#include "Care/LebtControl.h"

using namespace std;
using namespace repair_executor;


RepairExecutor::RepairExecutor(int argc, char** argv)
{
  ros::init(argc, argv, "RepairExecutor");

  this->nh = new ros::NodeHandle();
  this->loopRate = new ros::Rate(1);


  this->repairCmdTopic = "repair_action";
  this->repairCmdSub = this->nh->subscribe(this->repairCmdTopic, 1000, &RepairExecutor::RepairActionCallback, this);

  this->ownId = supplementary::SystemConfig::GetOwnRobotID();
  ROS_INFO("own robot Id: %d\n", this->ownId);

  //load all declared plugin objects
  this->repairPluginClassLoader = new pluginlib::ClassLoader<gen_repair_plugins::BaseRepair>("rosha_repair_executor", "gen_repair_plugins::BaseRepair");
  vector<string> declaredPlugins =  this->repairPluginClassLoader->getDeclaredClasses();

  for (int i=0; i<declaredPlugins.size(); i++)
  {
    try
    {
      gen_repair_plugins::BaseRepair* pluginItem = this->repairPluginClassLoader->createClassInstance(declaredPlugins[i]);
      pluginItem->initialize();

      //create the loop up table
      map<unsigned short, gen_repair_plugins::BaseRepair*>::iterator it;
      it = this->lookUp_IdtoPlugin.find(pluginItem->repairType); //check if the key already in map
      if (it == this->lookUp_IdtoPlugin.end() )
      {
        //not found, not yet inserted
        this->lookUp_IdtoPlugin.insert( std::pair<unsigned short, gen_repair_plugins::BaseRepair*>(pluginItem->repairType, pluginItem));
        ROS_INFO("registered Plugin in LookUp: %s", pluginItem->GetName().c_str());
      }
      else
      {
        ROS_ERROR("Key already exists in the lookup table.");
      }

    }
    catch(pluginlib::PluginlibException& ex)
    {
      string p = declaredPlugins[i];
      ROS_ERROR("The plugin %s failed to load for some reason. Error: %s", p.c_str(), ex.what());
    }
  }


}

RepairExecutor::~RepairExecutor() {

  //delete all new vars
  delete this->repairPluginClassLoader;
  delete this->loopRate;
  delete this->nh;
}

void RepairExecutor::Start()
{
  while(ros::ok()) {
    ros::spinOnce();
    this->loopRate->sleep();
  }
}


void RepairExecutor::RepairActionCallback(const rosha_msgs::RepairAction::ConstPtr& msg)
{
  ROS_INFO("received msg for robotId: %d, repairAction: %d, compName: %s, compId: %d",
           msg->robotId, msg->repairActionToPerform, msg->compName.c_str(), msg->compId);

  //check if the msg is local
  if (msg->robotId != this->ownId)
  {
    ROS_INFO("... ignore non local messages. msg id: %d != ownid: %d", msg->robotId, this->ownId);
    return;
  } else {
    //silent
  }

  HandleFailureType(msg->repairActionToPerform, msg->compId, msg->compName);

}


inline void RepairExecutor::HandleFailureType(int repairAction, int compId, string compName)
{
  //need comp info and action ... dummy just call repair plugin

  //use parameter space to "register" the pugins to a repair action ... problem with the c# behavior, not supported!
  //or use enum here ... all plugin-packagas needs to depend on the RepairAction msg, SimBase as well ... as repair ID

  //need to identify/find the matched plugin the vector of registered plugins ... forech ... is static in runtime ... calc fix mapping. -> loop up table!

  //map find faster than vector for bigger amount of elements (binary tree)
  map<unsigned short, gen_repair_plugins::BaseRepair*>::iterator iter;
  iter = this->lookUp_IdtoPlugin.find(repairAction);
  if (iter != this->lookUp_IdtoPlugin.end() )
  {
    //set the data
    iter->second->SetData(compId, compName, this->ownId);
    iter->second->Repair();
  }
  else
  {
    ROS_ERROR("Error while accessing the plugin for repair id: %d. The corresponding plugin seems not to be registered.", repairAction);
  }

  return;
}

