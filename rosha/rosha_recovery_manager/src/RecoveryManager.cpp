/*
 * RecoveryManager.cpp
 *
 *  Created on: May 8, 2014
 *      Author: dominik
 */

#include "../include/rosha_recovery_manager/RecoveryManager.h"

using namespace value_iteration;
using namespace std;

RecoveryManager::RecoveryManager()
{

  //create the mdp problem
  MDP* mdp = new MDP();
  GridMDP exampleGrid4x3(mdp);
  mdp->PrintMDP();

  this->mdp = mdp;

  //find the policy
  ValueIteration* v = new ValueIteration(mdp);
  v->SetConvergenceThreshold(0.0001);
  v->SetDiscountFactor(1.0);
  v->Plan();

  cout << "planning took ms: "<< v->GetPlanningDuration() << endl;
  cout << "needed iterations: " << v->GetIterations() << endl;
  v->PrintUtilities();
  v->PrintPolicy();

  this->vi = v;

}

RecoveryManager::~RecoveryManager()
{
  // TODO Auto-generated destructor stub
  delete this->mdp;
  delete this->vi;
}

int RecoveryManager::GetAction(int stateId)
{
  State* s = mdp->FindState(stateId);
  int actionId = vi->GetPolicy()[s];
  //send/execute action
  cout << "... send actions: " << actionId << " for state input: " << stateId << endl;

  return actionId;
}

