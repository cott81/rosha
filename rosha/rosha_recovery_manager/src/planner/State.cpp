/*
 * State.cpp
 *
 *  Created on: May 6, 2014
 *      Author: dominik
 */

#include "../include/planner/State.h"

using namespace value_iteration;

State::State(int id, string name)
{
  this->id = id;
  this->name = name;
  this->reward = 0;
}

void State::AddTransition(State* to, Action* a, double probability)
{
  Transition t(this, to, a, probability);
  AddTransition(&t);

}

void  State::AddTransition(Transition* transition)
{
  //add transition ordered by actions in the state object

  //check if key already exists
  int actionId = transition->GetAction()->GetId();
  if ( actionTransitions.find(actionId) == actionTransitions.end() )
  {
    // not found ... create the vector and add the transition
    vector<Transition*> v;
    v.push_back(transition);
    this->actionTransitions[actionId] = v;
  }
  else
  {
    // found ... vector already exists, check if an identical transition exists else add the transition
    for (vector<Transition*>::iterator it=actionTransitions[actionId].begin(); it < actionTransitions[actionId].end(); it++)
    {
      if (( (*it)->GetOrigin()->GetId() == transition->GetOrigin()->GetId() ) &&
          ( (*it)->GetDestination()->GetId() == transition->GetDestination()->GetId() ) &&
          ( (*it)->GetAction()->GetId() == transition->GetAction()->GetId() ) )
      {
        cout << "State: transition: <" << transition->GetOrigin()->GetName()<<"->"<< transition->GetDestination()->GetName()
            << " action: " << transition->GetAction()->GetName()
            << "> already in list" << endl;
        return;
      }
    }

    this->actionTransitions[actionId].push_back(transition);
  }


}
