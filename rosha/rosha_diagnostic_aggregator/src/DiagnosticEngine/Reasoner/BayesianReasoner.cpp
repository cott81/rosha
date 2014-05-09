/*
 * BayesianReasoner.cpp
 *
 * Copyright 2013 Carpe Noctem, Distributed Systems Group,
 * University of Kassel. All right reserved.
 *
 * The code is derived from the software contributed to Carpe Noctem by
 * the Carpe Noctem Team.
 *
 * The code is licensed under the Carpe Noctem Userfriendly BSD-Based
 * License (CNUBBL). Redistribution and use in source and binary forms,
 * with or without modification, are permitted provided that the
 * conditions of the CNUBBL are met.
 *
 * You should have received a copy of the CNUBBL along with this
 * software. The license is also available on our website:
 * http://carpenoctem.das-lab.net/license.txt
 *  Created on: Feb 5, 2013		6:04:12 PM
 *      Author: Dominik Kirchner
 */

#include <DiagnosticEngine/Reasoner/BayesianReasoner.h>

using namespace diagnostic_engine;
using namespace std;

BayesianReasoner::BayesianReasoner() {

  this->CLASSNAME = "BayesianReasoner";
  this->querySet = new vector<BayesianQueryItem>();
}

BayesianReasoner::~BayesianReasoner() {
  delete this->querySet;
  this->querySet = NULL;
}

//
//visitors for a bayesian reasoning
//

void BayesianReasoner::Visit(DSLModel& model) {

  DSL_network* net = model.GetDSLNetwork();

  // use clustering algorithm
  net->SetDefaultBNAlgorithm(DSL_ALG_BN_LAURITZEN);

  // update the network
  net->UpdateBeliefs();

  int slices = net->GetNumberOfSlices();      //total slices
  int currentSliceIndex = model.GetCurrentSliceIndex();

  vector<BayesianQueryItem>::iterator iter;
  for (iter = this->querySet->begin(); iter != this->querySet->end(); iter++) {
    // get the result value
    DSL_Dmatrix* matrix;
    int nodeHandle = iter->GetNodeId();
    matrix = net->GetNode(nodeHandle)->Value()->GetMatrix();
    const char* name = net->GetNode(nodeHandle)->GetId();

    DSL_idArray *theNames;
    theNames = net->GetNode(nodeHandle)->Definition()->GetOutcomesNames();

    int stateIndex;
    if (iter->GetStateId() < 0) {
      //invalid stateId ... choose simply the first
      ROS_WARN("%s: Visit(): invalid stateId %d! Choose simply the first (index 0) instead.", this->CLASSNAME, iter->GetStateId());
      stateIndex = 0;
    } else {
      //specified stateId ... maybe check if it is in range ... not here ...
      stateIndex = iter->GetStateId();
    }

    //for a DBN 2: dimension 0 is the time (time slices), dimension 1 are the states
    int dimensionOfMatrix = matrix->GetNumberOfDimensions();  //BUG: dimension of the matrix is alway 2 -> need num of states


    int numStates = 0;
    int timeSlices = 0; // = matrix->GetSizeOfDimension(0);


    int matrixIndex = 0;
    int maxIndex = 0;
//    cout << "matrix->GetSizeOfDimension(0) = " << matrix->GetSizeOfDimension(0) <<  "  matrix->GetSizeOfDimension(1) = " << matrix->GetSizeOfDimension(1) << endl;
    // twisted outputs -> bn matrix->GetSizeOfDimension(0)=numStates matrix->GetSizeOfDimension(1)=0;
    // twisted outputs -> dbn matrix->GetSizeOfDimension(0)=timeSlices matrix->GetSizeOfDimension(1)=numStates;
//    matrix->GetSizeOfDimension(1) == 0
    if(slices == 0) {
      // bayesian network
      numStates = matrix->GetSizeOfDimension(0);
      timeSlices = 1;

      matrixIndex = stateIndex;
      maxIndex = numStates - 1; //begin from 0
    } else {
      // dynamic bayesian network
      numStates = matrix->GetSizeOfDimension(1);
      timeSlices = matrix->GetSizeOfDimension(0);
      //count index increases column after column
      matrixIndex = currentSliceIndex * numStates + stateIndex;
      maxIndex = numStates * timeSlices;
      if (timeSlices > slices) {
        ROS_ERROR("%s: Visit(): time slice %d is bigger than max time window %d. Skip.", this->CLASSNAME, timeSlices, slices);
        continue;
      }
    }

    if (matrixIndex > maxIndex) {
      ROS_ERROR("%s: Visit(): matrix (array) index: %d is bigger than max index: %d. Skip. Check history settings =< model time slice settings.", this->CLASSNAME, matrixIndex, maxIndex);
      continue;
    }

    ROS_DEBUG("%s: Visit(DSLModel): dimensionOfVar: %d, currentSliceIndex: %d, stateIndex: %d -> matrixIndex: %d",
              this->CLASSNAME, numStates, currentSliceIndex, stateIndex, matrixIndex);


    //TODO: index check is needed! current time index is a "free" parameter
    double P_rootCause_FailureIsPresent = (*matrix)[matrixIndex];
    iter->SetStateProp(P_rootCause_FailureIsPresent);

    ROS_DEBUG("%s: Visit(DSLModel): t=%d P(\"%s\" = \"%s\") = %f\n", this->CLASSNAME, currentSliceIndex, name, (*theNames)[stateIndex], iter->GetStateProp());
  }

  return;
}

void BayesianReasoner::SetQuery(vector<int>* nodeHandles) {

  //implicitly query FIRST state ...
  ROS_WARN("%s: SetQuery(vector<int>* nodeHandles): No states of interest given. Query always first state of notes!");

  //set the query nodeHandles
  this->querySet->clear();

  for (int i=0; i<nodeHandles->size(); i++) {
    //add all queries
//    this->querySet->insert(std::pair<int, double>( (*nodeHandles)[i], -1.0));

    BayesianQueryItem queryItem;
    queryItem.SetNodeId((*nodeHandles)[i]);
    this->querySet->push_back(queryItem);

  }
  return;
}

//map <nodeId, stateId>
void BayesianReasoner::SetQuery(map<int, int>* nodeHandleStates) {

  //set the query nodeHandles
  this->querySet->clear();

  map<int, int>::iterator iter;
//  for (int i=0; i<nodeHandleStates->size(); i++) {
   for(iter = nodeHandleStates->begin(); iter != nodeHandleStates->end(); iter++) {
    //add all queries
//    this->querySet->insert(std::pair<int, double>( (*nodeHandles)[i], -1.0));

    BayesianQueryItem queryItem;
    queryItem.SetNodeId(iter->first);
    queryItem.SetStateId(iter->second);
    this->querySet->push_back(queryItem);
  }
  return;
}

void BayesianReasoner::SetQuery(std::vector<BayesianQueryItem>* query) {

  //set the query nodeHandles
  this->querySet = query;

  return;
}

std::vector<BayesianQueryItem>* BayesianReasoner::GetResults() {

  return this->querySet;
}


