/*
 * Plugins.cpp
 *
 *  Created on: Jun 5, 2014
 *      Author: dominik
 */

#include <pluginlib/class_list_macros.h>

#include <diagnostic_aggregator/analyzer.h>
#include <analyzer_plugins/MagicCube_analyzer.h>

PLUGINLIB_EXPORT_CLASS(vrep_magic_cube_analyzer_plugins::MagicCubeAnalyzer, diagnostic_aggregator::Analyzer)


