/*
 * RemoteSlamProcessing.h
 *
 *  Created on: Jun 11, 2014
 *      Author: dominik
 */

#ifndef REMOTESLAMPROCESSING_H_
#define REMOTESLAMPROCESSING_H_

#include "ros/ros.h"
#include <ros/package.h>

#include <GenRepairPlugins/BaseRepair.h>
#include <rosha_msgs/RepairAction.h>

namespace vrep_slam_repair_plugins
{

class RemoteSlamProcessing : public gen_repair_plugins::BaseRepair
{
public:
  RemoteSlamProcessing();
  virtual ~RemoteSlamProcessing();
  void Initialize(void** data, int length);
  void Repair();
  std::string GetName() {return this->pluginName;}

  void SetData(const rosha_msgs::RepairAction::ConstPtr& msg)
  {
    this->ownId = msg->robotId;
    this->targetCompId = msg->compId;
    this->targetCompName = msg->compName;
    this->failedRobotId = msg->failedRobotId;
  }

private:
  std::string pluginName;
  std::string packagePath;
  const std::string corrsepondingCompName;
  const int REPAIR_MSG_DELAY;
  std::string pathedModelFilename;
  int failedRobotId;
};

} /* namespace vrep_localization_repair_plugins */

#endif /* REMOTESLAMPROCESSING_H_ */
