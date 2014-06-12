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
#include "SystemConfig.h"
#include <pluginlib/class_loader.h>

//msg not tested
//#include <RepairExecuter/RepairActionCmd.h>
#include <rosha_msgs/RepairAction.h>

#include <GenRepairPlugins/BaseRepair.h>

namespace repair_executor {

class RepairExecutor {

public :
  RepairExecutor(int argc, char** argv);
  ~RepairExecutor();
  void Start();

private:
  ros::NodeHandle* nh;
  ros::Rate* loopRate;

  std::string repairCmdTopic;
  ros::Subscriber repairCmdSub;

  int ownId;

  std::map<unsigned short, gen_repair_plugins::BaseRepair*> lookUp_IdtoPlugin;

  pluginlib::ClassLoader<gen_repair_plugins::BaseRepair>* repairPluginClassLoader;

  void RepairActionCallback(const rosha_msgs::RepairAction::ConstPtr& msg);

  inline void HandleFailureType(int repairAction, int compId, std::string compName);


};

}



#endif /* REPAIREXECUTER_H_ */
