/*
 * ErrorSeederLib.h
 *
 *  Created on: May 28, 2014
 *      Author: dominik
 */

#ifndef ERRORSEEDERLIB_H_
#define ERRORSEEDERLIB_H_

#include <thread>
#include <mutex>
#include <random>

#include "ros/ros.h"
#include "error_seeder_msgs/Error.h"
#include "error_seeder_msgs/ErrorConf.h"
#include "Defs.h"

namespace error_seeder
{

std::mutex mutex;

class ErrorSeederLib
{
public:
  ErrorSeederLib(int ownCompId);
  virtual ~ErrorSeederLib();

  //void TestCallback(const std_msgs::Int32::ConstPtr& msg);

private:
  ros::NodeHandle nh;
  ros::Subscriber errorSub;
  ros::Subscriber errorConfSub;
  //ros::Subscriber testSub;
  int ownCompId;
  bool elFlag;
  void ErrorTriggerCallback(const error_seeder_msgs::Error::ConstPtr& msg);
  void ErrorConfCallback(const error_seeder_msgs::ErrorConf::ConstPtr& msg);

  bool runSpinLoop;
  void Spin();

  bool runRandTrigger;
  std::thread* randErrorThread;

  //! array of failure occurance rate for:
  //! null pointer, array index, deadlock, endless loop, general exception  failures
  //! (in this order (1/s)
  const static int NUM_FAILURES = 5;
  double failureRates [NUM_FAILURES];

  //! random number generator
  //std::default_random_engine generator;
  std::random_device rd;
  std::mt19937 gen;

  //! uniform distribution
  std::uniform_real_distribution<> uniformDistribution;

protected:
  void TriggerError(int errorId);
  void StartRandErrorTrigger();



};

} /* namespace error_seeder */

#endif /* ERRORSEEDERLIB_H_ */
