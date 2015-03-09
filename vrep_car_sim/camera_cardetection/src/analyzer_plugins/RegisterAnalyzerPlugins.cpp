/*
 * Plugins.cpp
 *
 *  Created on: Jun 5, 2014
 *      Author: dominik
 */

#include <pluginlib/class_list_macros.h>

#include <diagnostic_aggregator/analyzer.h>
#include <analyzer_plugins/camera_cardetection_QBFD_analyzer.h>

PLUGINLIB_EXPORT_CLASS(camera_cardetection_analyzer_plugins::Camera_CarDetection_QBFD_Analyzer, diagnostic_aggregator::Analyzer)


