/*
 * RemoteSlamProcessing.h
 *
 *  Created on: Jun 11, 2014
 *      Author: dominik
 */

#ifndef REMOTESLAMPROCESSING_H_
#define REMOTESLAMPROCESSING_H_

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
  std::string GetName() {return this->PLUGIN_NAME;}

  void SetData(const rosha_msgs::RepairAction::ConstPtr& msg)
  {
    this->ownId = msg->robotId;
    this->targetCompId = msg->compId;
    this->targetCompName = msg->compName;
    this->failedRobotId = msg->failedRobotId;
  }

private:
  const std::string PLUGIN_NAME;
  std::string packagePath;
  std::string pathedModelFilename;
  int failedRobotId;
};

} /* namespace vrep_localization_repair_plugins */

#endif /* REMOTESLAMPROCESSING_H_ */
