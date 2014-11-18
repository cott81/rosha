/*
 * Plugins.cpp
 *
 *  Created on: Jun 5, 2014
 *      Author: dominik
 */

#include <pluginlib/class_list_macros.h>

#include <diagnostic_aggregator/analyzer.h>
#include <analyzer_plugins/vrep_slam_QBFD_analyzer.h>
#include <analyzer_plugins/vrep_slam_adapter_QBFD_analyzer.h>
#include <analyzer_plugins/vrep_slam_remote_QBFD_analyzer.h>

PLUGINLIB_EXPORT_CLASS(vrep_slam_analyzer_plugins::VrepSlam_QBFD_Analyzer, diagnostic_aggregator::Analyzer)
PLUGINLIB_EXPORT_CLASS(vrep_slam_analyzer_plugins::VrepSlamAdapter_QBFD_Analyzer, diagnostic_aggregator::Analyzer)
PLUGINLIB_EXPORT_CLASS(vrep_slam_analyzer_plugins::VrepSlamRemote_QBFD_Analyzer, diagnostic_aggregator::Analyzer)


