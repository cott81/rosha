/*
 * GPS_SLAM_Recompose.h
 *
 *  Created on: Jun 11, 2014
 *      Author: dominik
 */

#ifndef GPSSLAMRECOMPOSE_H_
#define GPSSLAMRECOMPOSE_H_

#include <sstream>
#include <ros/package.h>

#include <GenRepairPlugins/BaseRepair.h>
#include <rosha_msgs/RepairAction.h>
#include <std_msgs/String.h>

#include <SystemConfig.h>

namespace car_localization_repair_plugins
{

class GPS_SLAM_Recompose : public gen_repair_plugins::BaseRepair
{
public:
  GPS_SLAM_Recompose();
  virtual ~GPS_SLAM_Recompose();
  void Initialize(void** data, int length);
  void Repair();
  std::string GetName() {return this->pluginName;}

private:
  std::string pluginName;
  const std::string corrsepondingCompName;
  std::string pathedModelFilename;

  ros::Publisher diagDeactivatePup;

  char hostname[128];
  std::string robotname;
  std::string fullName;
};

} /* namespace vrep_localization_repair_plugins */

#endif /* GPSSLAMRECOMPOSE_H_ */
