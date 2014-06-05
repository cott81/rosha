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

  //this->lebtControlPup = nh->advertise<Care::LebtControl>("LebtControl", 1000); //call Care to do some repair stuff

  //load all declared plugin objects
  this->repairPluginClassLoader = new pluginlib::ClassLoader<gen_repair_plugins::BaseRepair>("rosha_repair_executor", "gen_repair_plugins::BaseRepair");
  vector<string> declaredPlugins =  this->repairPluginClassLoader->getDeclaredClasses();

  for (int i=0; i<declaredPlugins.size(); i++)
  {
    try
    {
      gen_repair_plugins::BaseRepair* pluginItem = this->repairPluginClassLoader->createClassInstance(declaredPlugins[i]);
      pluginItem->initialize();
      this->genericRepairPlugins.push_back(pluginItem);
      ROS_INFO("registered Plugin: %s", pluginItem->GetName().c_str());
      //this->genericRepairPlugins[i]->Repair(); //resgistered output
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
  //delete this->repairPluginClassLoader;
  delete this->loopRate;
  delete this->nh;
}

void RepairExecutor::Start()
{
  while(ros::ok()) {
    ros::spinOnce();
    //this->genericRepairPlugins[0]->Repair();
    this->loopRate->sleep();
  }
}


void RepairExecutor::RepairActionCallback(const rosha_msgs::RepairAction::ConstPtr& msg)
{
  ROS_INFO("received msg for robotId: %d, repairAction: %d, compName: %s, compId: %d",
           msg->robotId, msg->repairActionToPerform, msg->compName.c_str(), msg->compId);

  /*

  //need comp info and action ... dummy just call repair plugin
  if (msg->data == dummyMapperRedundancyRepair) {
    cout << "data match" << endl;
    //cout << "size " << this->genericRepairPlugins.size() << endl;

    if (this->genericRepairPlugins.size() > 0 && this->genericRepairPlugins[dummyMapperRedundancyRepair] != NULL) {
      this->genericRepairPlugins[0]->Repair();
    } else {
      ROS_ERROR("no repair actions are defined. Can't do anything.");
    }

  } else if (msg->data == capFailureReport) {
    ROS_INFO("do dummy restart ...");
    //hard coded !!!
    Care::LebtControl m;
    m.receiverID = 53;
    m.ProcessAction = 11; // use enum from msg... here stop
    m.processID = 4;
    Care::BundleMd5 bundleHashMsg;
    char data[16] = {14, 251, 87, 121, 71, 128, 223, 19, 251, 89, 2, 132, 23, 95, 20, 145};
    for (int i= 0; i<16; i++) {
      bundleHashMsg.bundleHash[i] = data[i];
    }

    m.currentBundle = bundleHashMsg;

    //TODO: infos needed hash code need

    cout << "stop cmd" << endl;
    this->lebtControlPup.publish(m);


    Care::LebtControl mm;
    mm.receiverID = 53;
    mm.ProcessAction = 10; // use enum from msg... here stop
    mm.processID = 4;


    mm.currentBundle = bundleHashMsg;

    ros::Duration(1.0).sleep();

    //TODO: infos needed hash code need
    cout << "start cmd" << endl;
    this->lebtControlPup.publish(mm);

    //directly start ...

  } else if (msg->data == restartRepair) {

    cout <<"restart repair " << endl;

  } else if (msg->data == ifaceRestart) {
    cout <<"restart repair " << endl;
    system("sudo ~/work/impera/cn-care-ros-pkg/RepairExecuter/ifupdown.sh");
  } else if (msg->data == ifaceRestart) {
    cout <<"restart interface " << endl;

  } else if (msg->data == testRepair) {
    cout <<"TEST REPAIR " << endl;

    //BOESE BOESE but is working
    //system("echo \"2know_it\" | sudo -S ~/work/impera/cn-care-ros-pkg/RepairExecuter/ifupdown.sh");
    system("sudo ~/work/impera/cn-care-ros-pkg/RepairExecuter/ifupdown.sh");


  } else {
    ROS_ERROR("can't intertrept the data");
  }

  */
}
