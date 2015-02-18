/*
 * Plugins.cpp
 *
 *  Created on: Jun 5, 2014
 *      Author: dominik
 */

#include <pluginlib/class_list_macros.h>

#include <diagnostic_aggregator/analyzer.h>
#include <analyzer_plugins/Car_analyzer.h>

PLUGINLIB_EXPORT_CLASS(car_system_analyzer_plugins::CarSystemAnalyzer, diagnostic_aggregator::Analyzer)


