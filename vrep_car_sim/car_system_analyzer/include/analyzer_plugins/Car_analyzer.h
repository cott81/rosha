#ifndef CAR_SYSTEM_ANALYZER_H
#define CAR_SYSTEM_ANALYZER_H

//#include "DiagnosticAnalyzers/ObservationQueue.h"
//#include <AnalyzerHelper/ObservationQueue.h>
#include <diagnostic_aggregator/analyzer.h>
#include <diagnostic_aggregator/status_item.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <pluginlib/class_list_macros.h>

#include <DiagnosticEngine/DiagnosticEngine.h>
#include <DiagnosticEngine/Reasoner/BayesianQueryItem.h>


//#include <DkiHelpers/DataLogger.h>

#include <ros/ros.h>
#include <ros/package.h>
#include <string>
#include <sstream>
#include <iostream>
#include <unistd.h>

#include <climits>      //max double limit

//boost libs
#include <boost/thread/mutex.hpp>
#include <boost/algorithm/string/predicate.hpp>

#include <smile_lib/smile.h>
#include <SystemConfig.h>


namespace car_system_analyzer_plugins {


class CarSystemAnalyzer : public diagnostic_aggregator::Analyzer
{
public:
  CarSystemAnalyzer();

  ~CarSystemAnalyzer();

  bool init(const std::string base_name, const ros::NodeHandle &n);

  bool match(const std::string name);

  bool analyze(const boost::shared_ptr<diagnostic_aggregator::StatusItem> item);

  std::vector<boost::shared_ptr<diagnostic_msgs::DiagnosticStatus> > report();

  std::string getPath() const { return path_; }

  std::string getName() const { return nice_name_; }

private:

  const char* CLASSNAME;
  char hostname[128];
  std::string robotname;
  std::string fullName;
  std::string channelName;
//  dki_helpers::DataLogger* logger;
  int robotId;
  std::string robotIdString;

  boost::shared_ptr<diagnostic_aggregator::StatusItem> report_item_;

  diagnostic_engine::DiagnosticEngine* de;
  DSL_network* theNet;
  long modelId;

  diagnostic_engine::BayesianReasoner* reasoner;
  std::vector<diagnostic_engine::BayesianQueryItem> detailedQueryRequest;
  std::vector<diagnostic_engine::BayesianQueryItem>* reasoningResults;


  std::string path_, nice_name_;
  std::string nodeToAnalyze;
  XmlRpc::XmlRpcValue systemNodesToAnalyze;
  XmlRpc::XmlRpcValue systemNodesStateToAnalyze;
  bool has_initialized_;

  boost::mutex mutexObj;

};

}
#endif //CAR_SYSTEM_ANALYZER_H
