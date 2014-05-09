/*
 * State.h
 *
 *  Created on: May 6, 2014
 *      Author: dominik
 */

#ifndef STATE_H_
#define STATE_H_

#include "Transition.h"

#include <iostream>
#include <string>
#include <vector>
#include <map>

using namespace std;

namespace value_iteration
{

class State
{
public:
  State(int id, string name); //:graph::Vertex(id, name)
  void AddTransition(State* to, Action* a, double probability);
  void AddTransition(Transition* transition);

  int GetId() {return this->id;}
  string GetName() {return this->name;}

  void SetReward(double reward) {this->reward = reward;}
  double GetReward() {return this->reward;}
  map<int, vector<Transition*> > GetActionTransitions() {return this->actionTransitions;}

private:

  int id;
  string name;
  map<int, vector<Transition*> > actionTransitions;
  double reward;


};
}

#endif /* STATE_H_ */
