/*
 * Plugins.cpp
 *
 *  Created on: Jun 5, 2014
 *      Author: dominik
 */

#include <pluginlib/class_list_macros.h>

#include <diagnostic_aggregator/analyzer.h>
#include <analyzer_plugins/car_slam_QBFD_analyzer.h>
#include <analyzer_plugins/car_slam_adapter_QBFD_analyzer.h>
#include <analyzer_plugins/car_slam_remote_QBFD_analyzer.h>

PLUGINLIB_EXPORT_CLASS(car_slam_analyzer_plugins::CarSlam_QBFD_Analyzer, diagnostic_aggregator::Analyzer)
PLUGINLIB_EXPORT_CLASS(car_slam_analyzer_plugins::CarSlamAdapter_QBFD_Analyzer, diagnostic_aggregator::Analyzer)
PLUGINLIB_EXPORT_CLASS(car_slam_analyzer_plugins::CarSlamRemote_QBFD_Analyzer, diagnostic_aggregator::Analyzer)


