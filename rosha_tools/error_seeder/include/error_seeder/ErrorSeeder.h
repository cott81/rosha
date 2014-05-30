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

#include "error_seeder_msgs/Error.h"
#include "error_seeder_msgs/ErrorConf.h"

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


private:
  ros::NodeHandle* n;
  ros::Rate* pub_rate;
  ros::Publisher errorPub;
  ros::Publisher errorConfPub;
  //ros::Subscriber errorFeedbackSub;

  int compId;
  ErrorId errorId;

  /*!
   * \brief: handles the user input and checks the input
   *
   * asks the user for the ID of a component and the error Id to trigger. Checks if the user inputs
   * are valid (range of an int)
   * \params compId: pointer to an int, for returning the componend id
   * \params errorId: pointer to an int, for returning the errorId to trigger
   */
  void HandleUserInteraction();

  /*!
   * \brief checks if the component id is valid
   *
   * \params string s: input to convert
   * \params int& retVal: return value
   *
   * \return indicates if the given string was a valid component id (return: true)
   */
  bool IsValidId(std::string s, int& retVal);

  /*!
   * \brief checks if the given input string is a valid failure type
   *
   * \params string errorShortcut: input string
   * \params ErrorId& errorId: return value of the enum type ErrorId
   *
   * \return indicates if the given string was a valid failure type(return: true)
   */
  bool IsValidErrorId (std::string errorShortcut, ErrorId& errorId);

  /*!
   * \brief checks if the given input string was a valid command
   *
   * \params string command: the command string
   */
  bool IsValidCommand (std::string command);

  /*!
   * \brief Checks if the given input string was a valid probability
   *
   * Checks if the string was a valid probability. Needs to be a pure number and is checked that the
   * number is between 0 and 1.
   *
   * \param strings: input string
   * \param double& retVal: converted probability value
   *
   * \return indicates if the given input string was a valid probability (return true)
   */
  bool IsValidProb(std::string s, double& retVal);

  /*!
   * \brief Splits the given string, Uses a given deliminator for this.
   *
   * \param: string s: inout string to be spitted
   * \param: char delim: deliminator charactor
   * \param: vector<string>& elems: vector of splitted strings as result
   *
   * \return vector of splitted strings as result. identical to the 3. argument
   */
  std::vector<std::string> SplitString(const std::string &s, char delim, std::vector<std::string> &elems);

  /*!
   * \brief Sends the error msg.
   */
  void SendErrorMsg();

  /*!
   * \brief Sends a config message to tell the error_lib to configure the failure probabilities
   */
  void SendErrorConfMsg(double errorProb);

  /*!
   * \brief Prints a help text on the console.
   */
  void PrintHelp();

  /*!
   * \brief: Callback for incoming messages of successfully triggered failures. Needed for depend failure probabilies. NOT YET IMPLEMENTD
   */
  void ErrorTriggerCallback(const error_seeder_msgs::Error::ConstPtr& msg);

};

} /* namespace error_seeder */

#endif /* ERRORSEEDER_H_ */
