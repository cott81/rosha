/*
 * DSLModel.cpp
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
 *  Created on: Jan 31, 2013		4:59:25 PM
 *      Author: Dominik Kirchner
 */

#include "DiagnosticEngine/KnowledgeBase/DSLModel.h"

using namespace diagnostic_engine;
using namespace std;

DSLModel::DSLModel() {

  this->CLASSNAME = "DSLModel";

  //not here to have the chance to change history time settings
  this->Init();

}

DSLModel::DSLModel(unsigned int currentTimeSlice, unsigned int numOfHistorySlices, unsigned int sliceTime) {

  this->CLASSNAME = "DSLModel";

  //not here to have the chance to change history time settings
  this->Init(currentTimeSlice, numOfHistorySlices, sliceTime);

}

DSLModel::~DSLModel() {
  delete this->dslModel;
}

//void DSLModel::Init(unsigned int currentTimeSlice=8, unsigned int numOfHistorySlices=10, unsigned int sliceTime=3000) {
void DSLModel::Init(unsigned int currentTimeSlice, unsigned int numOfHistorySlices, unsigned int sliceTime) {
  this->dslModel = new DSL_network();
  this->id = 0;
  this->currentSliceIndex = currentTimeSlice;

  this->numOfHistorySlices = numOfHistorySlices;
  this->sliceTime = sliceTime;

  this->obsQueue.Init(this->numOfHistorySlices, this->sliceTime);
}

void DSLModel::ReadModelFromFile(const std::string fileName) {

  ROS_INFO("%s: ReadModelFromFile: read model from file: %s", this->CLASSNAME, fileName.c_str());
  if (this->dslModel->ReadFile(fileName.c_str()) != DSL_OKAY) {
    ROS_ERROR("%s: ReadModelFromFile: no able to read the model file: %s",
              this->CLASSNAME, fileName.c_str());
    return;
  }
  this->id = GenerateHash(fileName.c_str(), fileName.length());

  //test function ...
  //Test();
  return;
}

void DSLModel::AddObservations(int nodeHandle,  unsigned long arrivalTime, int evidenceState) {

//  ROS_INFO("%s: AddObservations: DEBUG: evidence state: %d", this->CLASSNAME, evidenceState);

  this->obsQueue.AddItem(nodeHandle, arrivalTime, evidenceState);
}

void DSLModel::Update() {

  //set ordered observations in queue to model
  this->dslModel->ClearAllEvidence();
  int obsNode = 0;

//  this->obsQueue.PrintItems();

  //set all evidences to the corresponding nodes
  //iterate over observations map <nodeHandle, vector<obs_items> >
  map<int, vector<ObservationQueueItem> >::iterator iter;
  for (iter = this->obsQueue.observations.begin(); iter != this->obsQueue.observations.end(); iter++) {

    //check if nodeHanle is valid (negative)
    if (iter->first < 0) {
      ROS_WARN("%s, Update(): node handle is invalid: %d", this->CLASSNAME, iter->first);
      continue;
    }

    ROS_DEBUG("%s: Update():", this->CLASSNAME);
    ROS_DEBUG("%s: Update(): SetTemporalEvidence for nodeId: %d", this->CLASSNAME, iter->first);
    for (int obsCount=0; obsCount< iter->second.size(); obsCount++) {
      //sets evidence twice if matched to the same time slice
      //TODO: optimization: do not overwrite observation for the same node ...
      //... iterate list from end (most current), if evidence for time slice already set -> skip ... or mean value

      ROS_DEBUG("%s: Update(): set evidence for \t nodeId: %d, time slice: %d, state: %d",
               this->CLASSNAME, iter->first, this->currentSliceIndex - iter->second[obsCount].correspondingTimeSlice, iter->second[obsCount].evidenceState);

      this->dslModel->GetNode(iter->first)->Value()->SetTemporalEvidence(this->currentSliceIndex - iter->second[obsCount].correspondingTimeSlice ,
                                                                         iter->second[obsCount].evidenceState);
    }
  }
}

void DSLModel::UpdateTime(unsigned long currentTime) {

  ROS_DEBUG("%s UpdateTime: in DSLModel", this->CLASSNAME);
  //update in the observations queue the the current time -> update the relative delays -> the time slices
  this->obsQueue.UpdateRelativeDelay(currentTime);

  //update the evidences for the dslNet (BN) -> clear all evidences and reset them
  this->Update();
}

unsigned int DSLModel::GenerateHash(const char * string, size_t len) {

  //better static in Diagnostic Engine?
  //from stackoverflow ... thanks

    unsigned int hash = 0;
    for(size_t i = 0; i < len; ++i)
        hash = 65599 * hash + string[i];
    return hash ^ (hash >> 16);
}

void DSLModel::Test() {

  // use clustering algorithm
 dslModel->SetDefaultBNAlgorithm(DSL_ALG_BN_LAURITZEN);

  // update the network
  dslModel->UpdateBeliefs();

  int slices = dslModel->GetNumberOfSlices();      //total slices
  ROS_INFO("slices: %d", slices);


  // get the handle of node "Deadlock"
  int rootCause_dl = dslModel->FindNode("Deadlock");
  int rootCause_cr = dslModel->FindNode("Crash");

  ROS_INFO("handle of Deadlock: %d", rootCause_dl);
  ROS_INFO("handle of Crash: %d", rootCause_cr);

  // get the result value
  DSL_Dmatrix* matrix;
  matrix = this->dslModel->GetNode(rootCause_dl)->Value()->GetMatrix();

  DSL_idArray *theNames;
  theNames = dslModel->GetNode(rootCause_dl)->Definition()->GetOutcomesNames();
  int presentIndex = theNames->FindPosition("Present"); // should be 1 (starts from 1?)
  int dimensionOfVar = matrix->GetNumberOfDimensions();
  ROS_INFO("index of Present: %d dimension: %d", presentIndex, dimensionOfVar);
  int matrixIndex = this->currentSliceIndex * dimensionOfVar + presentIndex;

  double P_rootCause_dlIsPresent = (*matrix)[matrixIndex];
  ROS_INFO("P(\"Deadlock\" = Present) = %f\n",P_rootCause_dlIsPresent);

}
