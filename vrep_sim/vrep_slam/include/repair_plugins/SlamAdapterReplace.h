/*
 * SlamAdapterReplace.h
 *
 *  Created on: Jun 11, 2014
 *      Author: dominik
 */

#ifndef SLAMADAPTERREPLACE_H_
#define SLAMADAPTERREPLACE_H_

#include <exception>
#include "ros/ros.h"
#include <ros/package.h>


#include <GenRepairPlugins/BaseRepair.h>
#include <rosha_msgs/RepairAction.h>

namespace vrep_slam_repair_plugins
{

class SlamAdapterReplace : public gen_repair_plugins::BaseRepair
{
public:
  SlamAdapterReplace();
  virtual ~SlamAdapterReplace();
  void Initialize(void** data, int length);
  void Repair();
  std::string GetName() {return this->pluginName;}

private:
  std::string pluginName;
  std::string packagePath;
  const std::string corrsepondingCompName;
  const int REPAIR_MSG_DELAY;
};

} /* namespace vrep_localization_repair_plugins */

#endif /* SLAMADAPTERREPLACE_H_ */
