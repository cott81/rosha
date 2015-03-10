/*
 * Plugins.cpp
 *
 *  Created on: Jun 5, 2014
 *      Author: dominik
 */

#include <pluginlib/class_list_macros.h>

#include <GenRepairPlugins/BaseRepair.h>
//#include <repair_plugins/CompRedundancyRepair.h>
//#include <repair_plugins/LocalizerRedundantReplace.h>
#include <repair_plugins/GPS_SLAM_Recompose.h>

PLUGINLIB_EXPORT_CLASS(car_localization_repair_plugins::GPS_SLAM_Recompose, gen_repair_plugins::BaseRepair)


