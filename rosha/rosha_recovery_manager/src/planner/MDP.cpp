/*
 * MDP.cpp
 *
 *  Created on: May 6, 2014
 *      Author: dominik
 */

#include "../include/planner/MDP.h"

using namespace std;
using namespace value_iteration;

MDP::MDP()
{

}

void MDP::SetStates(vector<State*> states)
{
  this->states = states;
  return;
}

void MDP::AddState(int stateId, string stateName)
{
  State s (stateId, stateName);
  AddState(&s);
  return;
}

void MDP::AddState(State* s)
{
//  cout << "DEBUG: state to check: name: " << s->GetName() << " id: " << s->GetId() << endl;
  //cout << states[0]->GetId() << endl;

  for (vector<State*>::iterator it=states.begin(); it < states.end(); it++)
  {
//    cout << "DEBUG: state in list to check: name: " << (*it)->GetName() << " id: " << (*it)->GetId() << endl;
    if ((*it)->GetId() == s->GetId())
    {
//      cout << "DEBUG: id in list:" << (*it)->GetId() << " state id: " << s->GetId() << endl;
      cout << "state: " << s->GetName() << " with ID:"<< s->GetId() << " already in list" << endl;
      return;
    }
  }
  this->states.push_back(s);
  return;
}


void MDP::AddTransition(State* from, Action* a, State* to, double probability)
{
  Transition t(from, to, a, probability);
  AddTransition(&t);
}

void MDP::AddTransition(Transition* transition)
{
  //TODO check if the states already exist in the MDP

  //check if already existing
  for (vector<Transition*>::iterator it=transitions.begin(); it < transitions.end(); it++)
  {
    if (( (*it)->GetOrigin()->GetId() == transition->GetOrigin()->GetId() ) &&
        ( (*it)->GetDestination()->GetId() == transition->GetDestination()->GetId() ) &&
        ( (*it)->GetAction()->GetId() == transition->GetAction()->GetId() ) )
    {
      cout << "transition: <" << transition->GetOrigin()->GetName()<<"->"<< transition->GetOrigin()->GetName() << "> already in list" << endl;
      return;
    }
  }

  //add to the (unstructered) list of the MDP object (double storage)
  this->transitions.push_back(transition);

  //add transition to the state (structured)
  transition->GetOrigin()->AddTransition(transition);
}

void MDP::AddAction(Action* action)
{
  //TODO: check if already present
  this->actions.push_back(action);
}

void MDP::AddReward(Reward* r)
{
  for (unsigned int i=0; i<rewards.size(); i++)
  {
    if (r->GetState()->GetId() == rewards[i]->GetState()->GetId())
    {
      cout << "Reward for state " << r->GetState()->GetName() << " already in list!" << endl;
      return;
    }
  }

  this->rewards.push_back(r);

  //set the reward value in the state object
  r->GetState()->SetReward(r->GetValue());
}

void MDP::PrintMDP()
{
  cout << endl<< "States: " << endl;
  for (unsigned int i=0; i<this->states.size(); i++)
  {
    cout << this->states[i]->GetId() << " " << this->states[i]->GetName() << endl;
  }

  //actions
  cout << endl << "Actions: " << endl;
  for (unsigned int i=0; i<this->actions.size(); i++)
  {
    cout << actions[i]->GetId() << " " << actions[i]->GetName() << endl;
  }

  // Transitions
  cout << endl << "Transitions: " << endl;
  for (unsigned int i=0; i<this->transitions.size(); i++)
  {
    cout << this->transitions[i]->GetOrigin()->GetName() << " -> " << transitions[i]->GetDestination()->GetName()
        << " by " << transitions[i]->GetAction()->GetName()
        << " with prob: " << transitions[i]->GetProbability() << endl;
  }

  //rewards
  cout << endl << "Rewards: " << endl;
  for (unsigned int i=0; i<this->rewards.size(); i++)
  {
    cout << rewards[i]->GetState()->GetName() << "_" << rewards[i]->GetValue() << endl;
  }

}

State* MDP::FindState(int stateId)
{
  for(vector<State*>::iterator it=this->states.begin(); it != this->states.end(); ++it)
  {
    if ((*it)->GetId() == stateId)
    {
      return *it;
    }
  }

  return NULL;

}

// ... only if it is more than a line
//void MDP::RegisterTransitionInState(Transition* transition)
//{
//  transition->GetOriginState()->AddTransition();
//}



