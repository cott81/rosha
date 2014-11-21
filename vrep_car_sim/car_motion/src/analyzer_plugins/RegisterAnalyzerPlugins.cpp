/*
 * Plugins.cpp
 *
 *  Created on: Jun 5, 2014
 *      Author: dominik
 */

#include <pluginlib/class_list_macros.h>

#include <diagnostic_aggregator/analyzer.h>
#include <analyzer_plugins/car_motion_QBFD_analyzer.h>

PLUGINLIB_EXPORT_CLASS(car_motion_analyzer_plugins::Car_Motion_QBFD_Analyzer, diagnostic_aggregator::Analyzer)


