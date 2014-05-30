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

ErrorSeederLib::ErrorSeederLib(int ownCompId) :
    uniformDistribution(0, 1), gen(rd())
{
  this->ownCompId = ownCompId;
  this->elFlag = true;
  this->runSpinLoop = true;
  this->randErrorThread = NULL;
  this->runRandTrigger = true;
  errorSub = nh.subscribe("/ErrorSeeder/ErrorTrigger", 1000, &ErrorSeederLib::ErrorTriggerCallback, this);
  errorConfSub = nh.subscribe("/ErrorSeeder/ErrorConf", 1000, &ErrorSeederLib::ErrorConfCallback, this);

  //default values of failure rates (1/s)
  this->failureRates[0] = 0.001; //np MTTF=1000s
  this->failureRates[1] = 0.001; //ar
  this->failureRates[2] = 0.001; //dl
  this->failureRates[3] = 0.001; //el
  this->failureRates[4] = 0.001; //ex
}

ErrorSeederLib::~ErrorSeederLib()
{
  delete this->randErrorThread;
  this->randErrorThread = NULL;
}

void ErrorSeederLib::ErrorTriggerCallback(const error_seeder_msgs::Error::ConstPtr& msg)
{
  if (msg->compId != this->ownCompId)
  {
    return;
  }
  ROS_INFO("received error trigger msg: compId: %d, errorId: %d", msg->compId, msg->errorId);

  TriggerError(msg->errorId);
}

void ErrorSeederLib::ErrorConfCallback(const error_seeder_msgs::ErrorConf::ConstPtr& msg)
{
  if (msg->compId != this->ownCompId)
  {
    return;
  }
  ROS_INFO("received error conf msg: compId: %d, errorId: %d, errorProb: %d", msg->compId, msg->errorId,
           msg->errorProb);

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
    ROS_ERROR("Type of failure not supported: %d", msg->errorId);
  }
}

void ErrorSeederLib::TriggerError(int errorId)
{
  if (errorId == NULLPOINTER)
  {
    ROS_INFO("trigger null pinter failure");

    int* a = NULL;
    int b = *a;
  }
  else if (errorId == ARRAYINDEXERROR)
  {
    ROS_INFO("trigger arrayindex failure");

    int a[1];
    int b = a[2];
    a[3] = 123;
  }
  else if (errorId == ENDLESSLOOP)
  {
    ROS_INFO("trigger endless loop failure");

    this->elFlag = true;
    while (this->elFlag)
    {
      //needed to receive messages
      ros::spinOnce();
    }

  }
  else if (errorId == STOP_ENDLESSLOOP)
  {
    ROS_INFO("stops endless loop");
    this->elFlag = false;
  }
  else if (errorId == DEADLOCK)
  {
    ROS_INFO("trigger deadlock");

    mutex.lock();
    //run a spin loop in an extra thread to stay responsible
    std::thread t(&ErrorSeederLib::Spin, this);
    mutex.lock();

    t.join();
  }
  else if (errorId == LEASEDEADLOCK)
  {
    ROS_INFO("leases deadlock");
    this->runSpinLoop = false;
    mutex.unlock();
  }
  else if (errorId == GENERAL_EXCEPTION)
  {
    ROS_INFO("trigger general exception");
    exception e;
    throw e;
  }
  else if (errorId == ENABLE_PROBABILITY)
  {
    ROS_INFO("start random error triggering due to given probs");

    //what happens if the object goes out of scope?
    randErrorThread = new std::thread(&ErrorSeederLib::StartRandErrorTrigger, this);

  }
  else if (errorId == DISABLE_PROBABILITY)
  {
    ROS_INFO("stop random error triggering");

    //stop the thread
    this->runRandTrigger = false;
    randErrorThread->join();

  }
  else
  {
    ROS_ERROR("Received failure id not supported: %d. Do nothing", errorId);
  }
  return;

}

void ErrorSeederLib::Spin()
{
  this->runSpinLoop = true;
  ros::NodeHandle tmp_nh;
  ros::Subscriber tmpSub = tmp_nh.subscribe("/ErrorSeeder/ErrorTrigger", 1000, &ErrorSeederLib::ErrorTriggerCallback,
                                            this);
  //choose a very slow rate to handle incoming messages not to intervere with rescource usage
  ros::Rate r(1.0);
  while (this->runSpinLoop)
  {
    ros::spinOnce();
    r.sleep();
  }
}

void ErrorSeederLib::StartRandErrorTrigger()
{
  //start own thread that triggers the error types dependend on the given probs
  this->runRandTrigger = true;
  double randVal_NP = -1.0, randVal_AR = -1.0, randVal_DL = -1.0, randVal_EL = -1.0, randVal_EX = -1.0;
  ros::Rate randTriggerRate(1.0);
  while (this->runRandTrigger)
  {
    // ... throw the dices ...
    randVal_NP = uniformDistribution(gen);
    randVal_AR = uniformDistribution(gen);
    randVal_DL = uniformDistribution(gen);
    randVal_EL = uniformDistribution(gen);
    randVal_EX = uniformDistribution(gen);

    ROS_DEBUG("failure occurance probs: NP: %f, AR: %f, DL: %f, EL: %f, EX: %f", randVal_NP, randVal_AR, randVal_DL,
              randVal_EL, randVal_EX);

    //FIXME: triggers error in this thread ... no blocking!
    // ... send error message that is handled by the main thread?
    // ... when deadly, close this thread

    //trigger only one failure at a time! ...
    if (randVal_NP < this->failureRates[0])
    {
      TriggerError(NULLPOINTER);
    }
    else if (randVal_AR < this->failureRates[1])
    {
      TriggerError(ARRAYINDEXERROR);
    }
    else if (randVal_DL < this->failureRates[2])
    {
      TriggerError(DEADLOCK);
    }
    else if (randVal_EL < this->failureRates[3])
    {
      TriggerError(ENDLESSLOOP);
    }
    else if (randVal_EX < this->failureRates[4])
    {
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
