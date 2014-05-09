/*
 * GridMDP.h
 *
 *  Created on: May 6, 2014
 *      Author: dominik
 */

#ifndef GRIDMDP_H_
#define GRIDMDP_H_

#include "../planner/State.h"
#include "../planner/Transition.h"
#include "../planner/Action.h"
#include "../planner/Reward.h"
#include "../planner/MDP.h"

#include <iostream>
#include <string>
#include <sstream>
#include <vector>
#include <map>

using namespace std;

namespace value_iteration
{

class GridPoint
{
public:
  GridPoint(int col, int row);

  int GetCol() {return this->col;}
  int GetRow() {return this->row;}

private:

  int col;
  int row;
};


class GridMDP
{
public:
  GridMDP(MDP* mdp);

private:
  bool IsGridCellValid(int col, int row);
  bool IsGridCellTerminal(int col, int row, double& reward);
  void AddActionTransitions(MDP* mdp, State* from, int stateId, Action* action, int* stateModifiers, double* transProbs, unsigned int length);

  vector<GridPoint*> invalidGridCells;  //inner walls
  map<GridPoint*, double> terminalGridCells; //terminal cells, with none default reward
};


}




#endif /* GRIDMDP_H_ */
