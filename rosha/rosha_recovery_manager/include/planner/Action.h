/*
 * Action.h
 *
 *  Created on: May 6, 2014
 *      Author: dominik
 */

#ifndef ACTION_H_
#define ACTION_H_

#include <string>

using namespace std;

namespace value_iteration
{
class Action
{
public:
  Action(int id, string name);

  int GetId() {return this->id;}
  string GetName() {return this->name;}

private:
  int id;
  string name;

};
}

#endif /* ACTION_H_ */
