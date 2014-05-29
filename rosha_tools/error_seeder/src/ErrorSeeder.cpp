/*
 * ErrorSeeder.cpp
 *
 *  Created on: May 28, 2014
 *      Author: dominik
 */

#include "../include/error_seeder/ErrorSeeder.h"

using namespace std;

namespace error_seeder
{

ErrorSeeder::ErrorSeeder(int argc, char** argv)
{
  ros::init(argc, argv, "ErrorSeeder");
  n = new ros::NodeHandle();
  pub_rate = new ros::Rate(5.0);

  //compId = -1;
  //errorId = UNDEFINED;


  //errorPub = n->advertise<std_msgs::Float64>("/ErrorSeeder/ErrorTrigger", 1000);
  errorPub = n->advertise<error_seeder_msgs::Error>("/ErrorSeeder/ErrorTrigger", 1000);
  //errorFeedbackSub = n->subscribe("/ErrorSeeder/ErrorFeedback", 1000, &ErrorSeeder::ErrorFeedbackCallback, this);

  //temp only!!!
  //ErrorSeederLib esl(n);

  //ros::Subscriber errorSub = n->subscribe("/ErrorSeeder/ErrorTrigger", 1000, &ErrorSeeder::ErrorTriggerCallback, this);

  //ros::Subscriber testSub = n->subscribe("/test123", 1000, &ErrorSeeder::TestCallback, this);
   //testSub = n->subscribe("/test123", 1000, &ErrorSeeder::TestCallback, this);

}

ErrorSeeder::~ErrorSeeder()
{
  delete n;
  delete pub_rate;
}

void ErrorSeeder::Start()
{
  cout << endl;
  cout << "Enter commands:" << endl;
  cout << "\t e.g. the component ID and the type of error Id -> 1:NP " << endl;

  //bring in the lib: sub object must live during the process
  //int ownId = 1;
  //ErrorSeederLib esl(ownId);

  try
  {
    while (ros::ok())
    {
      ros::spinOnce();

      //user interaction
      HandleUserInteraction();


      //TODO: heart beat msg
      /*
      std_msgs::Float64 msg;
      msg.data = 3.0;
      errorPub.publish(msg);
      */

      pub_rate->sleep();
    }
  }
  catch (exception& e)
  {
    ROS_FATAL("vrep_motion_node node caught exception. Aborting. %s", e.what());
    ROS_BREAK();
  }

  return;
}

void ErrorSeeder::HandleUserInteraction()
{
  string s;

    cout << "command:> ";
    cin >> s;
    IsValidCommand(s);

  return;
}

bool ErrorSeeder::IsValidCommand(string command)
{
  bool valid = false;

  //static commands
  if (command.compare("h") == 0 || command.compare("help") == 0)
  {
    //print help screen
  }
  else if (command.compare("q")==0 || command.compare("quit")==0 )
  {
    cout << " ... leave programm." << endl;
    //ros shutdown ... ???
    exit(0);
  }
  //
  //more to come
  //
  else
  {
    //dynamic command (like compId:errorType
    vector<string> elems;
    SplitString(command, ':', elems);

    if (elems.size() == 2)
    {
      //check each part
      //cout << elems[0] << " " << elems[1] << endl;

      if (IsValidId(elems[0], &compId) && IsValidErrorId(elems[1], &errorId) )
      {
        SendErrorMsg();
        valid = true;
      }
      else
      {
        cout << "input error with command: " << command << endl;
      }
    }
    else
    {
      cout << "\tcommand: "<< command << " not supported." << endl;
      //print help text
      valid = false;
    }
  }

  return valid;
}

std::vector<std::string> ErrorSeeder::SplitString(const std::string &s, char delim, std::vector<std::string> &elems)
{
    std::stringstream ss(s);
    std::string item;
    while (std::getline(ss, item, delim))
    {
        elems.push_back(item);
    }
    return elems;
}

bool ErrorSeeder::IsValidId(string s,  int* retVal)
{
  stringstream ss;
  ss << s;

  ss >> *retVal;
  if (ss.good())
  {
    //cout << ss.goodbit << " " << ss.eofbit << " " << ss.failbit << " " << ss.badbit << endl;
    //cout << ss.good() << " " << ss.eof() << " " << ss.fail() << " " << ss.bad() << endl;
    cerr << "no valid number." << endl << endl;
    return false;
  }
  else if (*retVal == 0 && s[0] != '0')
  {
    cerr << "no valid number 2." << endl << endl;
    return false;
  }
  else
  {
    //cout << *retVal << " is a number" << endl;
    if (*retVal > -1)
    {
      return true;
    }
    else
    {
      cerr << "needs to be bigger than 0." << endl << endl;
      return false;
    }
  }

  return false;
}

bool ErrorSeeder::IsValidErrorId(string errorShortcut, ErrorId* errorId)
{
  bool valid = true;;

  if (errorShortcut.compare("NP") == 0)
  {
    *errorId = NULLPOINTER;
  }
  else if (errorShortcut.compare("AR") == 0)
  {
    *errorId = ARRAYINDEXERROR;
  }
  else if (errorShortcut.compare("EL") == 0)
  {
    *errorId = ENDLESSLOOP;
  }
  else if (errorShortcut.compare("!EL") == 0)
  {
    *errorId = STOP_ENDLESSLOOP;
  }
  else if (errorShortcut.compare("DL") == 0)
  {
    *errorId = DEADLOCK;
  }
  else if (errorShortcut.compare("lDL") == 0)
  {
    *errorId = LEASEDEADLOCK;
  }
  else if (errorShortcut.compare("EX") == 0)
  {
    *errorId = GENERAL_EXCEPTION;
  }
  //
  // rest not yet supported
  //
  else
  {
    ROS_ERROR("Error type (%s) not supported %d", errorShortcut.c_str(), 1);
    valid = false;
  }



  return valid;
}

void ErrorSeeder::SendErrorMsg()
{
  //msg = ...
  error_seeder_msgs::Error msg;
  msg.compId = this->compId;
  msg.errorId = this->errorId;
  errorPub.publish(msg);
}

void ErrorSeeder::ErrorTriggerCallback(const error_seeder_msgs::Error::ConstPtr& msg)
{
  cout << "received msg .. " << endl;
}

void ErrorSeeder::TestCallback(const std_msgs::Int32::ConstPtr& msg)
{
  cout << "received msg .. " << endl;
}

} /* namespace error_seeder */

