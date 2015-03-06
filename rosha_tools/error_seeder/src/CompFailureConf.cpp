/*
 * CompFailureConf.cpp
 *
 *  Created on: Mar 6, 2015
 *      Author: dominik
 */

#include "../include/error_seeder/CompFailureConf.h"
#include "../include/error_seeder/Defs.h"

namespace error_seeder
{

CompFailureConf::CompFailureConf()
{
  // TODO Auto-generated constructor stub

  compId = -1;
  probNP = 0;
  probAR = 0;
  probDL = 0;
  probEL = 0;
  probEX = 0;

}

CompFailureConf::~CompFailureConf()
{
  // TODO Auto-generated destructor stub
}

bool CompFailureConf::IsProbValid(double prob)
{
  if (prob < 0.0 || prob > 1.0)
  {
    ROS_ERROR("Given prob %d is not valid. Do nothing.", prob);
    return false;
  }
  else
  {
    return true;
  }
}

void CompFailureConf::SetFailureProb(ErrorId failureId, double prob )
{
  if(!IsProbValid(prob) )
  {
    return;
  }

  if (failureId == NULLPOINTER)
  {
    probNP = prob;
  }
  else if (failureId == ARRAYINDEXERROR)
  {
    probAR = prob;
  }
  else if (failureId == DEADLOCK)
  {
    probDL = prob;
  }
  else if (failureId == ENDLESSLOOP)
  {
    probEL = prob;
  }
  else if (failureId == GENERAL_EXCEPTION)
  {
    probEX = prob;
  }
  else
  {
    ROS_WARN("Failure id is not known or supported. Do nothing.");
  }

  return;
}

double CompFailureConf::GetFailureProb(ErrorId failureId )
{
  if (failureId == NULLPOINTER)
  {
    return probNP;
  }
  else if (failureId == ARRAYINDEXERROR)
  {
    return probAR;
  }
  else if (failureId == DEADLOCK)
  {
    return probDL;
  }
  else if (failureId == ENDLESSLOOP)
  {
    return probEL;
  }
  else if (failureId == GENERAL_EXCEPTION)
  {
    return probEX;
  }
  else
  {
    ROS_ERROR("Failure id is not known or supported. Return -1.");
    return -1.0;
  }
}

} /* namespace error_seeder */
