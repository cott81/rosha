/*
 * RecoveryManager.h
 *
 *  Created on: May 8, 2014
 *      Author: dominik
 */

#ifndef RECOVERYMANAGER_H_
#define RECOVERYMANAGER_H_

#include "../planner/MDP.h"
#include "../planner/ValueInteration.h"
#include "../problems/GridMDP.h"


class RecoveryManager
{
public:
  RecoveryManager();
  virtual ~RecoveryManager();

  int GetAction(int stateId);

private:
  value_iteration::MDP* mdp;
  value_iteration::ValueIteration* vi;
};

#endif /* RECOVERYMANAGER_H_ */
