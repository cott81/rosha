/*
 * Reward.h
 *
 *  Created on: May 6, 2014
 *      Author: dominik
 */

#ifndef REWARD_H_
#define REWARD_H_

#include "State.h"

namespace value_iteration
{
class Reward
{
public:
  Reward(State* s, double value);

  double GetValue() {return this->value;}
  State* GetState() {return this->s;}

private:
  double value;
  State* s;

};
}




#endif /* REWARD_H_ */
