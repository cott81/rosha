/*
 * Action.cpp
 *
 *  Created on: May 6, 2014
 *      Author: dominik
 */

#include "../include/planner/Action.h"

using namespace std;
using namespace value_iteration;

Action::Action(int id, string name)
{
  this->id = id;
  this->name = name;
}


