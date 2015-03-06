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
  ROS_INFO("... start the failure simulator lib: component id: %d", ownCompId);
  this->ownCompId = ownCompId;
  this->elFlag = true;
  this->runSpinLoop = true;
  this->randErrorThread = NULL;
  this->runRandTrigger = true;
  this->isBlocked = false;
  errorSub = nh.subscribe("/ErrorSeeder/ErrorTrigger", 1000, &ErrorSeederLib::ErrorTriggerCallback, this);
  errorConfSub = nh.subscribe("/ErrorSeeder/ErrorConf", 1000, &ErrorSeederLib::ErrorConfCallback, this);

  //default values of failure rates (1/s)
  //this->failureRates[0] = 0.001; //np MTTF=1000s
  this->failureRates[0] = 0.1; //np MTTF=1000s
  this->failureRates[1] = 0.0; //ar
  this->failureRates[2] = 0.0; //dl
  this->failureRates[3] = 0.0; //el
  this->failureRates[4] = 0.0; //ex
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
  //ROS_INFO("received error conf msg: compId: %d, errorId: %d, errorProb: %f", msg->compId, msg->errorId,
          // msg->errorProb);

  for (int i=0; i<5; i++)
  {
  this->failureRates[i] = msg->errorProbs[i]; //copy, better to pass ref?
  //cout << "prob " << i << ": " << this->failureRates[i]<< endl;
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
    this->isBlocked = true;
    while (this->elFlag)
    {
      //needed to receive messages to release the endless loop again
      // problem for a component that simply receives and process msgs -> spinning still ok -> no blocking! -> no effect

      //ros::spinOnce(); // comment it, but no way to recover here
    }

  }
  else if (errorId == STOP_ENDLESSLOOP)
  {
    ROS_INFO("stops endless loop");

    //endlessloop was the cause for the blocking active
    if (this->elFlag == true)
    {
      this->isBlocked = false; //continue rand trigger loop when EP active
    }

    this->elFlag = false;

  }
  else if (errorId == DEADLOCK)
  {
    ROS_INFO("trigger deadlock");

    mutex.lock();
    //run a spin loop in an extra thread to stay responsible
    std::thread t(&ErrorSeederLib::Spin, this);
    this->isBlocked = true;
    mutex.lock();

    t.join();
  }
  else if (errorId == LEASEDEADLOCK)
  {
    ROS_INFO("leases deadlock");
    //not endless loop (-> deadlock) was the cause for the blocking
    if (this->elFlag == false)
    {
      this->isBlocked = false; //continue rand trigger loop when EP active
    }

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

    if (randErrorThread == NULL)
    {
      //what happens if the object goes out of scope?
      randErrorThread = new std::thread(&ErrorSeederLib::StartRandErrorTrigger, this);
    }
    else
    {
      ROS_INFO("error thread is already created. Do not create another one.");
    }

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
  //choose a very slow rate to handle incoming messages not to interfere with rescource usage
  ros::Rate r(1.0);
  while (this->runSpinLoop)
  {
    ros::spinOnce();
    r.sleep();
  }
}

void ErrorSeederLib::StartRandErrorTrigger()
{
  //send error trigger msg to own component (handled in main thread) to block the main thread
  ros::Publisher errorPub2 = nh.advertise<error_seeder_msgs::Error>("/ErrorSeeder/ErrorTrigger", 1000);
  error_seeder_msgs::Error msg;
  msg.compId = ownCompId;

  //start own thread that triggers the error types dependend on the given probs
  this->runRandTrigger = true;
  double randVal_NP = -1.0, randVal_AR = -1.0, randVal_DL = -1.0, randVal_EL = -1.0, randVal_EX = -1.0;
  ros::Rate randTriggerRate(1.0);
  while (this->runRandTrigger)
  {
    //do not try to trigger failures if the component is blocked (deadlock, endlessloop)
    if (isBlocked)
    {
      randTriggerRate.sleep();
      continue;
    }
    // ... throw the dices ...
    randVal_NP = uniformDistribution(gen);
    randVal_AR = uniformDistribution(gen);
    randVal_DL = uniformDistribution(gen);
    randVal_EL = uniformDistribution(gen);
    randVal_EX = uniformDistribution(gen);

    ROS_INFO("failure occurance probs: NP: %f, AR: %f, DL: %f, EL: %f, EX: %f", randVal_NP, randVal_AR, randVal_DL,
              randVal_EL, randVal_EX);

    //FIXME: triggers error in this thread ... no blocking!
    // ... send error message that is handled by the main thread?
    // ... when deadly, close this thread

    //trigger only one failure at a time! ...
    if (randVal_NP < this->failureRates[0])
    {
//      TriggerError(NULLPOINTER);
      msg.errorId = NULLPOINTER;
      errorPub2.publish(msg);
    }
    else if (randVal_AR < this->failureRates[1])
    {
//      TriggerError(ARRAYINDEXERROR);
      msg.errorId = ARRAYINDEXERROR;
      errorPub2.publish(msg);
    }
    else if (randVal_DL < this->failureRates[2])
    {
//      TriggerError(DEADLOCK);
      msg.errorId = DEADLOCK;
      errorPub2.publish(msg);
    }
    else if (randVal_EL < this->failureRates[3])
    {
//      TriggerError(ENDLESSLOOP);
      msg.errorId = ENDLESSLOOP;
      errorPub2.publish(msg);
    }
    else if (randVal_EX < this->failureRates[4])
    {
//      TriggerError(GENERAL_EXCEPTION);
      msg.errorId = GENERAL_EXCEPTION;
      errorPub2.publish(msg);
    }
    else
    {
      //no failure triggering
    }

    randTriggerRate.sleep();
  }

}

} /* namespace error_seeder */
