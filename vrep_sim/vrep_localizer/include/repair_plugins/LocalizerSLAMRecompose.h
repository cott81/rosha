/*
 * LocalizerSLAMRecompose.h
 *
 *  Created on: Jun 11, 2014
 *      Author: dominik
 */

#ifndef LOCALIZERSLAMRECOMPOSE_H_
#define LOCALIZERSLAMRECOMPOSE_H_

#include <sstream>
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
  std::string pathedModelFilename;

  ros::Publisher diagDeactivatePup;

  char hostname[128];
  std::string fullName;
};

} /* namespace vrep_localization_repair_plugins */

#endif /* LOCALIZERSLAMRECOMPOSE_H_ */
