/*
 * Reward.cpp
 *
 *  Created on: May 6, 2014
 *      Author: dominik
 */

#include "../include/planner/Reward.h"

using namespace value_iteration;

Reward::Reward(State* s, double value)
{
  this->s = s;
  this->value = value;
}



