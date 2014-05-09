/*
 * ValueInteration.h
 *
 *  Created on: May 7, 2014
 *      Author: dominik
 */

#ifndef VALUEINTERATION_H_
#define VALUEINTERATION_H_

#include <map>
#include <cmath>
#include <ctime>

#include "MDP.h"
#include "State.h"
#include "Action.h"
#include "Transition.h"

namespace value_iteration
{
class ValueIteration
{
public:
  ValueIteration(MDP* mdp);
  void Plan();

  void PrintPolicy();
  void PrintUtilities();

  std::map<State*, int> GetPolicy() {return this->policy;}

  void SetDiscountFactor (double discountFactor) {this->discountFactor=discountFactor;}
  double GetDiscountFactor () {return this->discountFactor;}

  void SetConvergenceThreshold(double convergenceThreshold) {this->convergenceThreshold=convergenceThreshold;}
  double GetConvergenceThreshold() {return this->convergenceThreshold;}

  int GetIterations() {return this->iteration;}
  double GetPlanningDuration() {return this->planningDuration;}

private:
  MDP* mdp;

  //utility for states
  std::map<State*, double>utilitites;

  //policy
  std::map<State*, int>policy;

  double discountFactor;
  double convergenceThreshold;
  double planningDuration;      //in ms

  int iteration;

};
}




#endif /* VALUEINTERATION_H_ */
