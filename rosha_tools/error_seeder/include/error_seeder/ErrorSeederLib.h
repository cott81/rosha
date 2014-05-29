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

#include "ros/ros.h"
#include "error_seeder_msgs/Error.h"
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
  //ros::Subscriber testSub;
  int ownCompId;
  bool elFlag;
  void ErrorTriggerCallback(const error_seeder_msgs::Error::ConstPtr& msg);

  bool runSpinLoop;
  void Spin();

  bool runRandTrigger;
  std::thread* randErrorThread;

protected:
  void TriggerError(int errorId);
  void TriggerRandomError();



};

} /* namespace error_seeder */

#endif /* ERRORSEEDERLIB_H_ */
