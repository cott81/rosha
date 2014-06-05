/*
 * Plugins.cpp
 *
 *  Created on: Jun 5, 2014
 *      Author: dominik
 */

#include <pluginlib/class_list_macros.h>

//#include <pluginlib_tutorials_/polygon_base.h>
#include "../../include/GenRepairPlugins/BaseRepair.h"
//#include <pluginlib_tutorials_/polygon_plugins.h>
#include "../../include/GenRepairPlugins/TestRepair.h"
#include "../../include/GenRepairPlugins/RestartRepair.h"
#include "../../include/GenRepairPlugins/CapFailureReport.h"

PLUGINLIB_EXPORT_CLASS(gen_repair_plugins::TestRepair, gen_repair_plugins::BaseRepair)
PLUGINLIB_EXPORT_CLASS(gen_repair_plugins::RestartRepair, gen_repair_plugins::BaseRepair)
PLUGINLIB_EXPORT_CLASS(gen_repair_plugins::CapFailureReport, gen_repair_plugins::BaseRepair)
//PLUGINLIB_EXPORT_CLASS(polygon_plugins::Square, polygon_base::RegularPolygon)


