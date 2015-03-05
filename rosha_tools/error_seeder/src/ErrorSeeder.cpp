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
  //resendThread_probMode = NULL;

  errorPub = n->advertise<error_seeder_msgs::Error>("/ErrorSeeder/ErrorTrigger", 1000);
  errorConfPub = n->advertise<error_seeder_msgs::ErrorConf>("/ErrorSeeder/ErrorConf", 1000);
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

  try
  {
    while (ros::ok())
    {
      ros::spinOnce();
      HandleUserInteraction();
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
  cin >> s; //blocking ... no problem because node does not receive any msgs
  IsValidCommand(s);

  return;
}

void ErrorSeeder::PrintHelp()
{
  cout <<"Help screen for ErrorSeeder:" << endl;
  cout << endl;
  cout << "\t h : prints this help text" << endl;
  cout << "\t q : ends the program" << endl;
  cout << "\t <compId>:<failureId> : sends a failure trigger msg, that should trigger a failure " << endl;
  cout << "\t\t with failureId in the component with id " << endl;
  cout << "\t\t supported failure types are: NP (nullpointer), AR (array index), DL (deadlock), EL (endless loop), EX (exception)" << endl;
  cout << "\t <compId>:EP : enables random triggering of failures for component with this id." << endl;
  cout << "\t <compId>:DP : disables random triggering for this component." << endl;
  cout << "\t <compId>:<failureId>:<failureProb> : configures the occurance probability of a failure for a component" << endl;
  cout << "\t\t by sending a error conf message to the component handled by the error lib" << endl;
  cout << endl;

}

bool ErrorSeeder::IsValidCommand(string command)
{
  bool valid = false;

  //static commands
  if (command.compare("h") == 0 || command.compare("help") == 0)
  {
    //print help screen
    PrintHelp();
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

      if (IsValidId(elems[0], compId) && IsValidErrorId(elems[1], errorId) )
      {
        SendErrorMsg();

        //
        //new threads that resent the x:EP periodically
        //
        if ( (errorId == ENABLE_PROBABILITY)
            &&  !(std::find(registeredResendCompIds.begin(), registeredResendCompIds.end(), compId) != registeredResendCompIds.end() )
            )
        {
          // need the current compId
          cout << "CREATE new THREAD" << endl;
          registeredResendCompIds.push_back(compId);
          resendThreadPool.push_back(new std::thread(&ErrorSeeder::ResendEPMessage, this, compId));
        }

        if ( errorId == DISABLE_PROBABILITY)
        {

          //check if the compId is registered, join thread, erase compId and thread from pool
          for (int i = 0; i < registeredResendCompIds.size(); i++)
          {
            if (registeredResendCompIds[i] == compId)
            {
              registeredResendCompIds.erase(registeredResendCompIds.begin() + i);
              resendThreadPool[i]->join();
              resendThreadPool.erase(resendThreadPool.begin() + i);
              break;
            }
          }

        }

        valid = true;
      }
      else
      {
        ROS_ERROR("Input error with command: %s", command.c_str());
      }
    }
    else if (elems.size() == 3)
    {
      //compId:failureId:occuranceProb  e.g. 1:NP:0.1
      //cout << elems[0] << " " << elems[1] << " " << elems[2] << endl;
      double errorTypeProb = 0;
      if (IsValidId(elems[0], compId) && IsValidErrorId(elems[1], errorId) && IsValidProb(elems[2], errorTypeProb) )
      {
        //send conf msg
        cout << "is valid -> send conf msg" << endl;
        SendErrorConfMsg(errorTypeProb);
      }
      else
      {
        ROS_ERROR("Input error with command: %s", command.c_str());
      }

    }
    else
    {
      ROS_WARN("Command not supported: %s", command.c_str());
      PrintHelp();
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

bool ErrorSeeder::IsValidId(string s,  int& retVal)
{
  stringstream ss;
  ss << s;

  ss >> retVal;
  if (ss.good())
  {
//    cout << ss.goodbit << " " << ss.eofbit << " " << ss.failbit << " " << ss.badbit << endl;
//    cout << ss.good() << " " << ss.eof() << " " << ss.fail() << " " << ss.bad() << endl;
    ROS_ERROR("no valid int number: %s\n", s.c_str());
    return false;
  }
  else if (retVal == 0 && s[0] != '0')
  {
    ROS_ERROR("no valid int number (2): %s\n", s.c_str());
    return false;
  }
  else
  {
    if (retVal > -1)
    {
      return true;
    }
    else
    {
      ROS_ERROR("Number needs to be bigger than 0");
      return false;
    }
  }

  return false;
}

bool ErrorSeeder::IsValidProb(string errorProbString, double & errorProb)
{
  bool valid = true;

  stringstream ss;
  ss << errorProbString;

  ss >> errorProb;
  if (ss.good())
  {
    ROS_ERROR("no valid double number: %s\n", errorProbString.c_str());
    return false;
  }
  else if (errorProb == 0 && errorProbString[0] != '0')
  {
    ROS_ERROR("no valid double number (2): %s\n", errorProbString.c_str());
    return false;
  }
  else
  {
    if ( (errorProb >= 0) && (errorProb <= 1) )
    {
      return true;
    }
    else
    {
      ROS_ERROR("Probability have to be between 0 and 1.");
      return false;
    }
  }

  return valid;
}

bool ErrorSeeder::IsValidErrorId(string errorShortcut, ErrorId& errorId)
{
  bool valid = true;;

  if (errorShortcut.compare("NP") == 0)
  {
    errorId = NULLPOINTER;
  }
  else if (errorShortcut.compare("AR") == 0)
  {
    errorId = ARRAYINDEXERROR;
  }
  else if (errorShortcut.compare("EL") == 0)
  {
    errorId = ENDLESSLOOP;
  }
  else if (errorShortcut.compare("!EL") == 0)
  {
    errorId = STOP_ENDLESSLOOP;
  }
  else if (errorShortcut.compare("DL") == 0)
  {
    errorId = DEADLOCK;
  }
  else if (errorShortcut.compare("lDL") == 0)
  {
    errorId = LEASEDEADLOCK;
  }
  else if (errorShortcut.compare("EX") == 0)
  {
    errorId = GENERAL_EXCEPTION;
  }
  else if (errorShortcut.compare("EP") == 0)
  {
    errorId = ENABLE_PROBABILITY;
  }
  else if (errorShortcut.compare("DP") == 0)
  {
    errorId = DISABLE_PROBABILITY;
  }
  //
  // rest not yet supported
  //
  else
  {
    ROS_ERROR("Failure type (%s) not supported %d", errorShortcut.c_str(), 1);
    valid = false;
  }

  return valid;
}

void ErrorSeeder::SendErrorMsg()
{
  error_seeder_msgs::Error msg;
  msg.compId = this->compId;
  msg.errorId = this->errorId;
  errorPub.publish(msg);
}

void ErrorSeeder::SendErrorConfMsg(double errorProb)
{
  error_seeder_msgs::ErrorConf msg;
  msg.compId = this->compId;
  msg.errorId = this->errorId;
  msg.errorProb = errorProb;
  errorConfPub.publish(msg);
}

void ErrorSeeder::ErrorTriggerCallback(const error_seeder_msgs::Error::ConstPtr& msg)
{
  ROS_INFO(".. error feedback for changing dependend failure not yet supported");
}


void ErrorSeeder::ResendEPMessage(int msgCompId)
{

  ros::Rate r(0.5);
  while (ros::ok())
  {

    //break if msgCompId is not any more in list ... very inefficient!!
    if (std::find(registeredResendCompIds.begin(), registeredResendCompIds.end(), msgCompId) == registeredResendCompIds.end()  )
    {
      // not any more registered
      cout << "BREAK" << endl;
      break;
    }

    cout << "loop"<< msgCompId << endl;

    error_seeder_msgs::Error msg;
    msg.compId = msgCompId;
    msg.errorId = ENABLE_PROBABILITY;

    errorPub.publish(msg);

    r.sleep();
  }

  return;
}



} /* namespace error_seeder */

