/*
 * SlamAdapterReplace.h
 *
 *  Created on: Jun 11, 2014
 *      Author: dominik
 */

#ifndef SLAMADAPTERREPLACE_H_
#define SLAMADAPTERREPLACE_H_

#include <sstream>
#include <exception>
#include <ros/package.h>

#include <GenRepairPlugins/BaseRepair.h>
#include <rosha_msgs/RepairAction.h>
#include <std_msgs/String.h>

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

  char hostname[128];
  std::string fullName;
  std::string pathedModelFilename;

  ros::Publisher diagDeactivatePup;
};

} /* namespace vrep_localization_repair_plugins */

#endif /* SLAMADAPTERREPLACE_H_ */
