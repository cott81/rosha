/*
 * Plugins.cpp
 *
 *  Created on: Jun 5, 2014
 *      Author: dominik
 */

#include <pluginlib/class_list_macros.h>

#include <diagnostic_aggregator/analyzer.h>
#include <analyzer_plugins/car_laser_driver_QBFD_analyzer.h>

PLUGINLIB_EXPORT_CLASS(car_laser_driver_analyzer_plugins::CarLaserDriver_QBFD_Analyzer, diagnostic_aggregator::Analyzer)


