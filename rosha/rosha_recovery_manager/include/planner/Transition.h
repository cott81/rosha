/*
 * Transition.h
 *
 *  Created on: May 6, 2014
 *      Author: dominik
 */

#ifndef TRANSITION_H_
#define TRANSITION_H_

//#include "../include/Action.h"
#include "Action.h"

//#include "../include/State.h"


namespace value_iteration
{
class State;

class Transition
{
public:
  Transition(State* from, State * to, Action* a, double probability);

  //shadows base methods ... good?
  State* GetOrigin() {return (State*) this->origin;}
  State* GetDestination() {return (State*) this->destination;}
  double GetProbability() {return this->probability;}
  Action* GetAction() {return this->a;}

private:
  //cast better as double data?
  State* origin;
  State* destination;
  Action* a;
  double probability;
};
}

#endif /* TRANSITION_H_ */
