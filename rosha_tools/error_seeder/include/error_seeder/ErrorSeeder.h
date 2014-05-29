/*
 * ErrorSeeder.h
 *
 *  Created on: May 28, 2014
 *      Author: dominik
 */

#ifndef ERRORSEEDER_H_
#define ERRORSEEDER_H_

#include "ros/ros.h"
#include <exception>

#include "std_msgs/Int32.h"
#include "std_msgs/Float64.h"
#include "error_seeder_msgs/Error.h"

#include "ErrorSeederLib.h"
#include "Defs.h"

namespace error_seeder
{

class ErrorSeeder
{
public:
  ErrorSeeder(int argc, char** argv);
  virtual ~ErrorSeeder();

  /*!
   * \brief: start the Error Seeders main loop, handles user inputs
   */
  void Start();
  /*!
   * \brief: handles the user input and checks the input
   *
   * asks hte user for the ID of a component and the error Id to trigger. Checks if the user inputs
   * are valid (range of an int)
   * \params compId: pointer to an int, for returning the componend id
   * \params errorId: pointer to an int, for returning the errorId to trigger
   */
  void HandleUserInteraction();

  void TestCallback(const std_msgs::Int32::ConstPtr& msg);

private:
  ros::NodeHandle* n;
  //ros::NodeHandle nh;
  ros::Rate* pub_rate;
  ros::Publisher errorPub;
  ros::Subscriber errorFeedbackSub;

  ros::Subscriber testSub;

  //error command
  int compId;
  ErrorId errorId;

  // BUG: build problem
  //void ErrorFeedbackCallback(const std_msgs::Int32::ConstPtr& msg);
  bool IsValidId(std::string s, int* retVal);
  bool IsValidErrorId (std::string errorShortcut, ErrorId* errorId);
  bool IsValidCommand (std::string command);
  std::vector<std::string> SplitString(const std::string &s, char delim, std::vector<std::string> &elems);
  void SendErrorMsg();

  void ErrorTriggerCallback(const error_seeder_msgs::Error::ConstPtr& msg);

};

} /* namespace error_seeder */

#endif /* ERRORSEEDER_H_ */
