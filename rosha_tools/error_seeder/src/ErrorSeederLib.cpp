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

ErrorSeederLib::ErrorSeederLib(int ownCompId) : uniformDistribution(0, 1), gen(rd())
{
  this->ownCompId = ownCompId;
  this->elFlag = true;
  this->runSpinLoop = true;
  this->randErrorThread = NULL;
  this->runRandTrigger = true;
  errorSub = nh.subscribe("/ErrorSeeder/ErrorTrigger", 1000, &ErrorSeederLib::ErrorTriggerCallback, this);
  errorConfSub = nh.subscribe("/ErrorSeeder/ErrorConf", 1000, &ErrorSeederLib::ErrorConfCallback, this);
  //errorFeedbackSub = nh.subscribe("/ErrorSeeder/ErrorFeedback", 1000, &ErrorSeeder::ErrorFeedbackCallback, this);

  //default values of failure rates (1/s)
  this->failureRates[0] = 0.001; //np MTTF=1000s
  this->failureRates[1] = 0.001; //ar
  this->failureRates[2] = 0.001; //dl
  this->failureRates[3] = 0.001; //el
  this->failureRates[4] = 0.001; //ex
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
  cout << "compId: " << msg->compId << " errorId: " << msg->errorId << endl;

  TriggerError(msg->errorId);
}

void ErrorSeederLib::ErrorConfCallback(const error_seeder_msgs::ErrorConf::ConstPtr& msg)
{
  if (msg->compId != this->ownCompId)
  {
    return;
  }

  cout << "received error conf msg .. " << endl;
  cout << "compId: " << msg->compId << " errorTypeId: " << msg->errorId << " errorProb: " << msg->errorProb << endl;

  if (msg->errorId == NULLPOINTER)
  {
    this->failureRates[0] = msg->errorProb;
  }
  else if (msg->errorId == ARRAYINDEXERROR)
  {
    this->failureRates[1] = msg->errorProb;
  }
  else if (msg->errorId == DEADLOCK)
  {
    this->failureRates[2] = msg->errorProb;
  }
  else if (msg->errorId == ENDLESSLOOP)
  {
    this->failureRates[3] = msg->errorProb;
  }
  else if (msg->errorId == GENERAL_EXCEPTION)
  {
    this->failureRates[4] = msg->errorProb;
  }
  else
  {
    cerr << "given type of failure ("<<msg->errorId<<") not supported! Do nothing." << endl;
  }

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
    randErrorThread = new std::thread (&ErrorSeederLib::StartRandErrorTrigger, this);

  }
  else if (errorId == DISABLE_PROBABILITY)
  {
    cout << "stop random error triggering" << endl;

    //stop the thread
    this->runRandTrigger = false;
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
  double randVal_NP = -1.0, randVal_AR=-1.0, randVal_DL=-1.0, randVal_EL=-1.0, randVal_EX=-1.0;
  ros::Rate randTriggerRate(1.0);
  while (this->runRandTrigger)
  {
    // ... throw the dices ...
    randVal_NP = uniformDistribution(gen);
    randVal_AR = uniformDistribution(gen);
    randVal_DL = uniformDistribution(gen);
    randVal_EL = uniformDistribution(gen);
    randVal_EX = uniformDistribution(gen);

    cout << "failure occurance probs: " <<
        " np: " << randVal_NP <<
        " ar: " << randVal_AR <<
        " dl: " << randVal_DL <<
        " el: " << randVal_EL <<
        " ex: " << randVal_EX <<
        endl;

    //trigger only one failure at a time! ...
      if (randVal_NP < this->failureRates[0])
      {
        cout << "trigger np failure" << endl;
        TriggerError(NULLPOINTER);
      }
      else if(randVal_AR < this->failureRates[1])
      {
        cout << "trigger ar failure" << endl;
        TriggerError(ARRAYINDEXERROR);
      }
      else if(randVal_DL < this->failureRates[2])
      {
        cout << "trigger dl failure" << endl;
        TriggerError(DEADLOCK);
      }
      else if(randVal_EL < this->failureRates[3])
      {
        cout << "trigger el failure" << endl;
        TriggerError(ENDLESSLOOP);
      }
      else if(randVal_EX < this->failureRates[4])
      {
        cout << "trigger ex failure" << endl;
        TriggerError(GENERAL_EXCEPTION);
      }
      else
      {
        //no failure triggering
      }


    randTriggerRate.sleep();
  }

}



} /* namespace error_seeder */
