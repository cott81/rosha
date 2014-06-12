/*
 * Plugins.cpp
 *
 *  Created on: Jun 5, 2014
 *      Author: dominik
 */

#include <pluginlib/class_list_macros.h>

#include <GenRepairPlugins/BaseRepair.h>
//#include <repair_plugins/CompRedundancyRepair.h>
#include <repair_plugins/LocalizerRedundantReplace.h>

//PLUGINLIB_EXPORT_CLASS(vrep_localization_repair_plugins::CompRedundancyRepair, gen_repair_plugins::BaseRepair)
PLUGINLIB_EXPORT_CLASS(vrep_localization_repair_plugins::LocalizerRedundantReplace, gen_repair_plugins::BaseRepair)


