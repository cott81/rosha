/*
 * ErrorSeederLib.cpp
 *
 *  Created on: May 28, 2014
 *      Author: dominik
 */

#include "../include/error_seeder/ErrorSeederLib.h"

using namespace std;

namespace error_seeder
{

ErrorSeederLib::ErrorSeederLib(int ownCompId)
{
  this->ownCompId = ownCompId;
  this->elFlag = true;
  this->runSpinLoop = true;
  this->randErrorThread = NULL;
  this->runRandTrigger = true;
  errorSub = nh.subscribe("/ErrorSeeder/ErrorTrigger", 1000, &ErrorSeederLib::ErrorTriggerCallback, this);
  //errorFeedbackSub = nh.subscribe("/ErrorSeeder/ErrorFeedback", 1000, &ErrorSeeder::ErrorFeedbackCallback, this);

}

ErrorSeederLib::~ErrorSeederLib()
{
  // TODO Auto-generated destructor stub
  delete this->randErrorThread;
  this->randErrorThread = NULL;
}

void ErrorSeederLib::ErrorTriggerCallback(const error_seeder_msgs::Error::ConstPtr& msg)
{
  if (msg->compId != this->ownCompId)
  {
    return;
  }

  cout << "received error trigger msg .. " << endl;
  cout << "compId: " << msg->compId << "errorId: " << msg->errorId << endl;

  TriggerError(msg->errorId);

}

void ErrorSeederLib::TriggerError(int errorId)
{
  if (errorId == NULLPOINTER)
  {
    cout << "trigger nullpointer" << endl;
    int* a = NULL;
    int b = *a;

    cout << "after np" << endl;
  }
  else if (errorId == ARRAYINDEXERROR)
  {
    cout << "trigger arrayindex error" << endl;

    //not deadly ... underminined behavior, because some arbitrary mem is changed
    int a[1];
    int b = a[2];
    a[3] = 123;
  }
  else if (errorId == ENDLESSLOOP )
  {
    cout << "trigger endless loop" << endl;
    this->elFlag = true;
    while(this->elFlag)
    {
      //needed to receive messages
      ros::spinOnce();
    }

  }
  else if (errorId == STOP_ENDLESSLOOP)
  {
    cout << "stop endless loop" << endl;
    this->elFlag = false;
  }
  else if (errorId == DEADLOCK)
  {
    cout << "trigger deadlock" << endl;

    //create a deadlock
    //create a second thread und start Blocking Func A
    //std::thread t (&ErrorSeederLib::BlockingFuncA, this);
    //BlockingFuncB();
    mutex.lock();
    cout << "here" << endl;
    //run a spin loop in an extra thread to stay responsible
    std::thread t (&ErrorSeederLib::Spin, this);
    mutex.lock();

    t.join();

  }
  else if (errorId == LEASEDEADLOCK)
  {
    cout << "lease deadlock" << endl;
    this->runSpinLoop = false;
    mutex.unlock();
  }
  else if (errorId == GENERAL_EXCEPTION)
  {
    cout << "trigger general exception" << endl;
    exception e;
    throw e;
  }
  else if (errorId == ENABLE_PROBABILITY)
  {
    cout << "start random error triggering due to given probs" << endl;

    //what happens if the object goes out of scope?
    randErrorThread = new std::thread (&ErrorSeederLib::TriggerRandomError, this);

  }
  else if (errorId == DISABLE_PROBABILITY)
  {
    cout << "stop random error triggering" << endl;

    //stop the thread
    randErrorThread->join();

  }
  else
  {
    cout << "errorId: " << errorId << " not known. Do nothing" << endl;
  }
  return;

}

void ErrorSeederLib::Spin()
{
  this->runSpinLoop = true;
  ros::NodeHandle tmp_nh;
  ros::Subscriber tmpSub = tmp_nh.subscribe("/ErrorSeeder/ErrorTrigger", 1000, &ErrorSeederLib::ErrorTriggerCallback, this);
  //choose a very slow rate to handle incoming messages not to intervere with rescource usage
  ros::Rate r(1.0);
  while (this->runSpinLoop)
  {
    cout << "spin" << endl;
    ros::spinOnce();
    r.sleep();
  }

}

void ErrorSeederLib::StartRandErrorTrigger()
{
  //start own thread that triggers the error types dependend on the given probs
  this->runRandTrigger = true;
  ros::Rate randTriggerRate(1.0);
  while (this->runRandTrigger)
  {
    // ... throw the dices ...

    //check the specified probabilities

    //trigger the errors

    randTriggerRate.sleep();
  }
}


} /* namespace error_seeder */
