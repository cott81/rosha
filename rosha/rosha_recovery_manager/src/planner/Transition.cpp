/*
 * Transition.cpp
 *
 *  Created on: May 6, 2014
 *      Author: dominik
 */

#include "../include/planner/Transition.h"

using namespace std;
using namespace value_iteration;

Transition::Transition(State* from, State * to, Action* a, double probability)
{
  this->a = a;
  this->destination = to;
  this->origin = from;
  this->probability = probability;
}


