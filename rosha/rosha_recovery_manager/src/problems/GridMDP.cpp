/*
 * GridMDP.cpp
 *
 *  Created on: May 6, 2014
 *      Author: dominik
 */

#include "../include/problems/GridMDP.h"

using namespace value_iteration;
using namespace std;


GridMDP::GridMDP(MDP* mdp)
{
  int rows = 3;
  int cols = 4;

  double defaultReward = -0.04;

  //define inner walls
  GridPoint w1(1,1);
  invalidGridCells.push_back(&w1);

  //define terminal cells
//  GridPoint goal(3,2);
  terminalGridCells.insert(std::pair<GridPoint*, double>(new GridPoint(3, 2), 1.0));
  terminalGridCells.insert(pair<GridPoint*, double>(new GridPoint(3,1), -1.0));


  int stateId = -1;
  string stateName ="not set";
  stringstream ss;

  // over all possible states, skip invalid ones
  for (int i=0; i<cols; i++)
  {
    for (int ii=0; ii<rows; ii++ )
    {
      if(IsGridCellValid(i, ii))
      {
        //
        //create state
        //
        stateId = 10*i + ii; // 01 02 ... 11 ... 22
        ss << "s";
        ss << stateId;
        stateName = ss.str();
        ss.str("");
        State* s = new State(stateId , stateName);
        mdp->AddState(s);

        //
        //create rewards
        //
        Reward* r;
//        double d = -81;
//        double* rewardValue = &d;
        double rewardValue = -81;
        if (IsGridCellTerminal(i, ii, rewardValue))
        {
          //set individual reward
//          cout << "TERMINAL STATE: set reward " << rewardValue << endl;
//          r = new Reward (s, *rewardValue); //better to use a pointer a change it later?
          r = new Reward (s, rewardValue); //better to use a pointer a change it later?
        }
        else
        {
          //set default reward
          r = new Reward (s, defaultReward);
        }
        mdp->AddReward(r);
      }
    }
  }

  //Actions /assumption actions are valid for all states
  mdp->AddAction(new Action(0, "up"));
  mdp->AddAction(new Action(1, "down"));
  mdp->AddAction(new Action(2, "left"));
  mdp->AddAction(new Action(3, "right"));

  //Transitions
  for (int i=0; i<cols; i++)
  {
    for (int ii=0; ii<rows; ii++ )
    {
//      double* reward;
//      double d = -81.0;
//      reward = &d;
      double reward = 81;
      if(IsGridCellValid(i, ii) && (!IsGridCellTerminal(i, ii, reward)) )
      {
        for (unsigned int actionIndex=0; actionIndex<mdp->GetActions().size(); actionIndex++)
        {
          int stateId = 10*i + ii;
          State* from = mdp->FindState(stateId);
          Action* action = mdp->GetActions()[actionIndex];

          //            int stateModifiers[3];
          //            double transProbs[3];
          switch (actionIndex)
          {
            case 0:
              //up (x,y) +1, -10, +10
            {
              int stateModifiers[] = {+1, -10, +10}; //{10, -1, +1};
              double transProbs[] = {0.8, 0.1, 0.1};
              AddActionTransitions(mdp, from, stateId, action, stateModifiers, transProbs, 3);
              break;
            }

            case 1:
              //down
            {
              int stateModifiers[] = {-1, +10, -10};
              double transProbs[] = {0.8, 0.1, 0.1};
              AddActionTransitions(mdp, from, stateId, action, stateModifiers, transProbs, 3);
              break;
            }
            case 2:
              //left
            {
              int stateModifiers[] = {-10, -1, +1};
              double transProbs[] = {0.8, 0.1, 0.1};
              AddActionTransitions(mdp, from, stateId, action, stateModifiers, transProbs, 3);
              break;
            }
            case 3:
              //right
            {
              int stateModifiers[] = {+10, +1, -1};
              double transProbs[] = {0.8, 0.1, 0.1};
              AddActionTransitions(mdp, from, stateId, action, stateModifiers, transProbs, 3);
              break;
            }

            default:
              cout << "case label (" << actionIndex<<") not found" << endl;
          }

        }
      }
    }
  }




  //remove states (walls)

  //set special rewards (for terminal states)
}

void GridMDP::AddActionTransitions(MDP* mdp, State* from, int stateId, Action* action,
                                   int* stateModifiers, double* transProbs, unsigned int length)
{
  double prob = 0;
  bool match = false;
  for (unsigned int sideEffect=0; sideEffect<length; sideEffect++)
  {
    State* to = mdp->FindState(stateId + stateModifiers[sideEffect]);

    if (to == NULL)
    {
      //state not found -> possible outside the grid or in inner wall
      // -> simple bounce back => transition in own state
      prob+=transProbs[sideEffect];
      match = true;
    }
    else
    {
      Transition* t = new Transition(from, to, action, transProbs[sideEffect]);
      mdp->AddTransition(t);
    }
  }

  if (match)
  {
    Transition* stay = new Transition(from, from, action, prob);
    mdp->AddTransition(stay);
  }

  return;
}

bool GridMDP::IsGridCellValid(int col, int row)
{
  for (vector<GridPoint*>::iterator it=invalidGridCells.begin(); it != invalidGridCells.end(); it++)
  {
    if ( ((*it)->GetCol() == col) && ( (*it)->GetRow()==row) )
    {
      //invalid
      return false;
    }
  }

  return true;
}

bool GridMDP::IsGridCellTerminal(int col, int row, double& reward)
{
  for (map<GridPoint*, double>::iterator it=terminalGridCells.begin(); it!=terminalGridCells.end(); it++)
  {
    if ( ( it->first->GetCol() == col) && ( it->first->GetRow()==row) )
    {
      //cell is terminal
      double d = it->second;
//      reward = &d;
      reward = d;

      return true;
    }
  }

  return false;
}
//
//class GridPoint
//

GridPoint::GridPoint(int col, int row)
{
  this->row = row;
  this->col = col;
}


