/*
 * ValueIteration.cpp
 *
 *  Created on: May 7, 2014
 *      Author: dominik
 */

#include "../include/planner/ValueInteration.h"

using namespace value_iteration;
using namespace std;

ValueIteration::ValueIteration(MDP* mdp)
{
  this->mdp = mdp;
  this->discountFactor = 1;
  this->convergenceThreshold = 0.0001;
  this->iteration = 0;
}

void ValueIteration::Plan()
{
  //time measurement
  clock_t begin = clock();

  double delta = convergenceThreshold + 1;
  double sum = 0, biggest_sum = 0;

  iteration = 0;
  while (delta > convergenceThreshold )
  {
    delta = 0;
    map<State*, double> utilitiesOld = this->utilitites;        //real copy?

    //for all states
    //    for (vector<State*>::iterator it=mdp->GetStates().begin(); it != mdp->GetStates().end(); ++it)
    for (unsigned int s_i=0; s_i < mdp->GetStates().size(); s_i++)
    {
      //      State* s = *it;
      State* s = mdp->GetStates()[s_i];
//      cout << "State: " << s->GetName() << endl;

      //u(s) = r(s) + discount * max_a SUM_s' P(s' | s, a) u(s')

      biggest_sum = -10000; //TODO min_double
      //for all actions ... better all actions for this state
      //      for (vector<Action*>::iterator a_it=mdp->GetActions().begin(); a_it != mdp->GetActions().end(); ++a_it)
      for (unsigned int a_i=0; a_i<this->mdp->GetActions().size(); a_i++)
      {
        //        Action* a = *a_it;
        Action* a = this->mdp->GetActions()[a_i];
//        cout << "\tAction: " << a->GetName()<<":"<<a->GetId()<<endl;

        //SUM over all (rechable) states
        sum = 0;
        //for all transitions from state s caused by action a


        //        for (vector<Transition*>::iterator t_it=s->GetActionTransitions()[a->GetId()].begin();
        //            t_it != s->GetActionTransitions()[a->GetId()].end();
        //            ++t_it
        //        )
        for (unsigned int t_i=0; t_i < s->GetActionTransitions()[a->GetId()].size(); t_i++)
        {
          //          Transition* t = *t_it;
          Transition* t = s->GetActionTransitions()[a->GetId()][t_i];
//          cout<<"\t\tTransition: "<< t->GetOrigin()->GetName()<<"->"<<t->GetDestination()->GetName()<<":"<<a->GetName()<< endl ;
          sum += t->GetProbability() * utilitiesOld[t->GetDestination()];
//          cout << "\t\tsum: " << sum << " = t_p:"<<t->GetProbability()<<" * u: " << utilitiesOld[t->GetDestination()]<< endl;
        }

//        cout << "\tsum: " << sum << endl;
        if (sum > biggest_sum)
        {
          biggest_sum = sum;
          //TODO ineffience
//          cout << "\tset policy("<<s->GetName()<<")="<<a->GetId()<<":"<<a->GetName()<<endl;
          this->policy[s] = a->GetId();
        }
      }

      this->utilitites[s] = s->GetReward() + discountFactor * biggest_sum; //check if the reward is really set
//      cout << "\tUtility: u("<<s->GetName()<<") = r:"<<s->GetReward()<<" + "<< discountFactor << " * " << biggest_sum << endl;

      //biggest change
      double current_delta = fabs(this->utilitites[s] -  utilitiesOld[s]);
      //find the biggest delta over the states
      if (delta < current_delta )
      {
        delta = current_delta;
      }
    } //for all states

    iteration++;
  }

  clock_t end = clock();
  this->planningDuration = double(end - begin) / CLOCKS_PER_SEC * 1000; //in ms

}

void ValueIteration::PrintUtilities()
{
	cout << endl << "Utilities: " << endl;
  for (map<State*, double>::iterator it=this->utilitites.begin(); it!=this->utilitites.end(); ++it)
  {
    //TODO present as grid???
    cout << it->first->GetName() << "\t" << it->second << endl;
  }
  return;
}

void ValueIteration::PrintPolicy()
{
	cout << endl << "Policy: " << endl;
  for (map<State*, int>::iterator it=this->policy.begin(); it!=this->policy.end(); ++it)
  {
    //TODO present as grid???
    cout << it->first->GetName() << "\t" << it->second << endl;
  }
  return;
}


