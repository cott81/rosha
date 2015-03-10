/*
 * Plugins.cpp
 *
 *  Created on: Jun 5, 2014
 *      Author: dominik
 */

#include <pluginlib/class_list_macros.h>

#include <diagnostic_aggregator/analyzer.h>
#include <analyzer_plugins/car_inertia_QBFD_analyzer.h>

PLUGINLIB_EXPORT_CLASS(car_inertia_analyzer_plugins::CarInertia_QBFD_Analyzer, diagnostic_aggregator::Analyzer)


