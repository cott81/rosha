/*
 * LocalizerRedundantReplace.h
 *
 *  Created on: Jun 11, 2014
 *      Author: dominik
 */

#ifndef LOCALIZERREDUNDANTREPLACE_H_
#define LOCALIZERREDUNDANTREPLACE_H_

#include "ros/ros.h"

#include <GenRepairPlugins/BaseRepair.h>
#include <rosha_msgs/RepairAction.h>

namespace vrep_localization_repair_plugins
{

class LocalizerRedundantReplace : public gen_repair_plugins::BaseRepair
{
public:
  LocalizerRedundantReplace();
  virtual ~LocalizerRedundantReplace();
  void initialize();
  void Repair();
  std::string GetName() {return this->pluginName;}

private:
  std::string pluginName;
  const std::string corrsepondingCompName;
  const int REPAIR_MSG_DELAY;
};

} /* namespace vrep_localization_repair_plugins */

#endif /* LOCALIZERREDUNDANTREPLACE_H_ */
