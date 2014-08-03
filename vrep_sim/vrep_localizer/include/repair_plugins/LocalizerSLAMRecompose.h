/*
 * LocalizerSLAMRecompose.h
 *
 *  Created on: Jun 11, 2014
 *      Author: dominik
 */

#ifndef LOCALIZERSLAMRECOMPOSE_H_
#define LOCALIZERSLAMRECOMPOSE_H_

#include "ros/ros.h"
#include <ros/package.h>

#include <GenRepairPlugins/BaseRepair.h>
#include <rosha_msgs/RepairAction.h>
#include <std_msgs/String.h>

namespace vrep_localization_repair_plugins
{

class LocalizerSLAMRecompose : public gen_repair_plugins::BaseRepair
{
public:
  LocalizerSLAMRecompose();
  virtual ~LocalizerSLAMRecompose();
  void Initialize(void** data, int length);
  void Repair();
  std::string GetName() {return this->pluginName;}

private:
  std::string pluginName;
  const std::string corrsepondingCompName;
  const int REPAIR_MSG_DELAY;
  std::string pathedModelFilename;

  ros::Publisher diagDeactivatePup;
};

} /* namespace vrep_localization_repair_plugins */

#endif /* LOCALIZERSLAMRECOMPOSE_H_ */
