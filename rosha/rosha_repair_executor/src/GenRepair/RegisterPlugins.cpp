/*
 * Plugins.cpp
 *
 *  Created on: Jun 5, 2014
 *      Author: dominik
 */

#include <pluginlib/class_list_macros.h>

#include "../../include/GenRepairPlugins/BaseRepair.h"
#include "../../include/GenRepairPlugins/RestartRepair.h"
#include <GenRepairPlugins/AddCommLink.h>
#include <GenRepairPlugins/RemoveCommLink.h>

PLUGINLIB_EXPORT_CLASS(gen_repair_plugins::RestartRepair, gen_repair_plugins::BaseRepair)
PLUGINLIB_EXPORT_CLASS(gen_repair_plugins::AddCommLink, gen_repair_plugins::BaseRepair)
PLUGINLIB_EXPORT_CLASS(gen_repair_plugins::RemoveCommLink, gen_repair_plugins::BaseRepair)


