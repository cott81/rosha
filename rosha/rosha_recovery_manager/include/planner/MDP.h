/*
 * MDP.h
 *
 *  Created on: May 5, 2014
 *      Author: dominik
 */

#ifndef MDP_H_
#define MDP_H_

#include "Transition.h"
#include "State.h"
#include "Action.h"
#include "Reward.h"

#include <vector>
#include <iostream>

using namespace std;

namespace value_iteration {

class MDP {

public:
  MDP();
  void SetStates(vector<State*> states);
  void AddState(int stateId, string stateName);
  void AddState(State* s);

  //  void SetActions(vector<Action> actions);
//  void SetTransitions(vector<Transition> transitions);
//
  void AddTransition(State* from, Action* a, State* to, double probability);       //P(s | a, s') = xy
  void AddTransition(Transition* transition);

  void AddAction(Action* action);
  void AddReward(Reward* r);

  void PrintMDP();

  vector<Action*> GetActions() {return this->actions;}
  vector<State*> GetStates() {return this->states;}

  State* FindState(int stateId);

private:

  void RegisterTransitionInState(Transition* transition);
  //TODO: create objects for states, actions, and transitions
  vector<State*> states;
  vector<Action*> actions; //how to represent state dependend actions sets actions(s)
  vector<Transition*> transitions;
  vector<Reward*> rewards;

};

}




#endif /* MDP_H_ */
