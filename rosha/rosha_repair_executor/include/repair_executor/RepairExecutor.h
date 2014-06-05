/*
 * RepairExecuter.h
 *
 *  Created on: Sep 5, 2012
 *      Author: dominik
 */

#ifndef REPAIREXECUTER_H_
#define REPAIREXECUTER_H_

#include <iostream>
#include <sstream>
#include <vector>

#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include <pluginlib/class_loader.h>

//msg not tested
//#include <RepairExecuter/RepairActionCmd.h>
#include <rosha_msgs/RepairAction.h>

#include <GenRepairPlugins/BaseRepair.h>

namespace repair_executor {

class RepairExecutor {

  enum repair_t {
    dummyMapperRedundancyRepair = 0,
    capFailureReport,
    restartRepair,
    ifaceRestart,
    testRepair = 100
  } repairPlugins;

public :
  RepairExecutor(int argc, char** argv);
  ~RepairExecutor();
  void Start();

private:
  ros::NodeHandle* nh;
  ros::Rate* loopRate;

  std::string repairCmdTopic;
  ros::Subscriber repairCmdSub;
  ros::Publisher lebtControlPup;

  std::vector<gen_repair_plugins::BaseRepair*> genericRepairPlugins;

  pluginlib::ClassLoader<gen_repair_plugins::BaseRepair>* repairPluginClassLoader;

  void RepairActionCallback(const rosha_msgs::RepairAction::ConstPtr& msg);


};

}



#endif /* REPAIREXECUTER_H_ */
