/*
 * Plugins.cpp
 *
 *  Created on: Jun 5, 2014
 *      Author: dominik
 */

#include <pluginlib/class_list_macros.h>

#include <GenRepairPlugins/BaseRepair.h>
#include <repair_plugins/SlamAdapterReplace.h>
#include <repair_plugins/RemoteSlamProcessing.h>

PLUGINLIB_EXPORT_CLASS(vrep_slam_repair_plugins::SlamAdapterReplace, gen_repair_plugins::BaseRepair)
PLUGINLIB_EXPORT_CLASS(vrep_slam_repair_plugins::RemoteSlamProcessing, gen_repair_plugins::BaseRepair)


