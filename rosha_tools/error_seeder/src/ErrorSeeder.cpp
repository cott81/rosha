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
  errorConfPub = n->advertise<error_seeder_msgs::ErrorConf>("/ErrorSeeder/ErrorConf", 1000);

  //errorFeedbackSub = n->subscribe("/ErrorSeeder/ErrorFeedback", 1000, &ErrorSeeder::ErrorFeedbackCallback, this);

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
        valid = true;
      }
      else
      {
        cout << "input error with command: " << command << endl;
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

bool ErrorSeeder::IsValidId(string s,  int& retVal)
{
  stringstream ss;
  ss << s;

  ss >> retVal;
  if (ss.good())
  {
//    cout << ss.goodbit << " " << ss.eofbit << " " << ss.failbit << " " << ss.badbit << endl;
//    cout << ss.good() << " " << ss.eof() << " " << ss.fail() << " " << ss.bad() << endl;
    cerr << "no valid number." << endl << endl;
    return false;
  }
  else if (retVal == 0 && s[0] != '0')
  {
    cerr << "no valid number 2." << endl << endl;
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
      cerr << "needs to be bigger than 0." << endl << endl;
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
    cerr << "no valid number." << endl << endl;
    return false;
  }
  else if (errorProb == 0 && errorProbString[0] != '0')
  {
    cerr << "no valid number 2." << endl << endl;
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
      cerr << "Probability have to be between 0 and 1." << endl << endl;
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
    ROS_ERROR("Error type (%s) not supported %d", errorShortcut.c_str(), 1);
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
  cout << "received msg .. " << endl;
}

void ErrorSeeder::TestCallback(const std_msgs::Int32::ConstPtr& msg)
{
  cout << "received msg .. " << endl;
}

} /* namespace error_seeder */

