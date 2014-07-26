#ifndef VREP_MOTION_ANALYZER_H
#define VREP_MOTION_ANALYZER_H

//#include "DiagnosticAnalyzers/ObservationQueue.h"
//#include <AnalyzerHelper/ObservationQueue.h>
#include <diagnostic_aggregator/analyzer.h>
#include <diagnostic_aggregator/status_item.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <pluginlib/class_list_macros.h>

#include <DiagnosticEngine/DiagnosticEngine.h>
#include <DiagnosticEngine/KnowledgeBase/BayesianKC.h>
#include <DiagnosticEngine/KnowledgeBase/BayesianML.h>
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


namespace vrep_motion_analyzer_plugins {


class VrepMotion_QBFD_Analyzer : public diagnostic_aggregator::Analyzer
{
public:
  VrepMotion_QBFD_Analyzer();

  ~VrepMotion_QBFD_Analyzer();

  bool init(const std::string base_name, const ros::NodeHandle &n);

  bool match(const std::string name);

  bool analyze(const boost::shared_ptr<diagnostic_aggregator::StatusItem> item);

  std::vector<boost::shared_ptr<diagnostic_msgs::DiagnosticStatus> > report();

  std::string getPath() const { return path_; }

  std::string getName() const { return nice_name_; }

private:

  const char* CLASSNAME;
  char hostname[128];
  std::string fullName;
  std::string channelName;
  //dki_helpers::DataLogger* logger;

  boost::shared_ptr<diagnostic_aggregator::StatusItem> report_item_;

  diagnostic_engine::DiagnosticEngine* de;
  DSL_network* theNet;
  long modelId;

  diagnostic_engine::BayesianReasoner* reasoner;

  //Knowledge Change object for all observers
  diagnostic_engine::BayesianKC* heartBeatIntervalKC;
  diagnostic_engine::BayesianKC* cpuKC;
  diagnostic_engine::BayesianKC* memKC;
  diagnostic_engine::BayesianKC* threadKC;
  diagnostic_engine::BayesianKC* streamKC_nl;
  diagnostic_engine::BayesianKC* streamKC_ex;
  diagnostic_engine::BayesianKC* msgFreqKC;

  std::vector<diagnostic_engine::BayesianQueryItem> detailedQueryRequest;
  std::vector<diagnostic_engine::BayesianQueryItem>* reasoningResults;

  std::string path_, nice_name_;
  std::string nodeToAnalyze;
  XmlRpc::XmlRpcValue systemNodesToAnalyze;
  XmlRpc::XmlRpcValue systemNodesStateToAnalyze;
  bool has_initialized_;
  int cpuUsage, memUsage, threadUsage;
  double streamRate_nl, streamRate_ex, msgFreq;
  int nodeHandle_CpuObs, nodeHandle_MemObs, nodeHandle_ThreadObs, nodeHandle_StreamObs_nl, nodeHandle_StreamObs_ex,
    nodeHandle_MsgFreqObs, nodeHandle_HeartBeatInterval;
  double cpuNodeStateLimits[2]; // = {200, 400};  //continues measurement -> discrete node state: e.g. 1% cpu -> state low
  double memNodeStateLimits[2]; // = {15500.0, 17000.0}; //in MB upper limit?
  int threadNodeStateLimits[2]; // = {11, 13}; //number of threads used
  double streamNodeStateLimits_nl[2]; // = {8.0, 10.0}; //output lines per s
  double streamNodeStateLimits_ex[1]; // = {0}; //rate of exception thrown in the stream per s
  double msgFreqNodeStateLimits[2]; // = {0.9, 1.1}; //message freq
  int heartBeatIntervalLimits[2]; // 900 1100

  int currentSliceIndex;     //Index of slice for t=0 (current evidence)
  unsigned long stampedTime;        //time from msg header in ms

  bool receivedMonUpdates; //flag that marks that monitoring updates have been received and processed
  std::vector<diagnostic_engine::KnowledgeChange*> evidences;

  boost::mutex mutexObj;

  bool linkRegistered;

};

}
#endif //VREP_MOTION_ANALYZER_H
