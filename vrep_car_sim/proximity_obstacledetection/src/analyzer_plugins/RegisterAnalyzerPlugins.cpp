/*
 * Plugins.cpp
 *
 *  Created on: Jun 5, 2014
 *      Author: dominik
 */

#include <pluginlib/class_list_macros.h>

#include <diagnostic_aggregator/analyzer.h>
#include <analyzer_plugins/proximity_obstacledetection_QBFD_analyzer.h>

PLUGINLIB_EXPORT_CLASS(proximity_obstacledetection_analyzer_plugins::Proximity_Obstacledetection_QBFD_Analyzer, diagnostic_aggregator::Analyzer)


