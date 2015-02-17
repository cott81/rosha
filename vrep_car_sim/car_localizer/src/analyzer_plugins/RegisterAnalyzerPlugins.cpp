/*
 * Plugins.cpp
 *
 *  Created on: Jun 5, 2014
 *      Author: dominik
 */

#include <pluginlib/class_list_macros.h>

#include <diagnostic_aggregator/analyzer.h>
#include <analyzer_plugins/car_localizer_QBFD_analyzer.h>

PLUGINLIB_EXPORT_CLASS(car_localizer_analyzer_plugins::Car_Localizer_QBFD_Analyzer, diagnostic_aggregator::Analyzer)


