/*
 * Plugins.cpp
 *
 *  Created on: Jun 5, 2014
 *      Author: dominik
 */

#include <pluginlib/class_list_macros.h>

#include <diagnostic_aggregator/analyzer.h>
#include <analyzer_plugins/vrep_laser_driver_QBFD_analyzer.h>

PLUGINLIB_EXPORT_CLASS(vrep_laser_driver_analyzer_plugins::VrepLaserDriver_QBFD_Analyzer, diagnostic_aggregator::Analyzer)


