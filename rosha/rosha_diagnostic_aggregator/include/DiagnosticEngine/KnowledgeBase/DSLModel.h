/*
 * DSLModel.h
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
 *  Created on: Jan 31, 2013		4:56:43 PM
 *      Author: Dominik Kirchner
 */

#ifndef BAYESIAN_COMP_MODEL_H_
#define BAYESIAN_COMP_MODEL_H_


#include <AnalyzerHelper/ObservationQueue.h>
#include <DiagnosticEngine/KnowledgeBase/Model.h>

#include "smile_lib/smile.h"
#include <ros/ros.h>

namespace diagnostic_engine  {

using namespace diagnostic_aggregator;

//object for model(BN) of one component
/*
 * Bayesian network from SMILE lib
 */
class DSLModel : public Model {

public:
  DSLModel();
  DSLModel(unsigned int currentTimeSlice, unsigned int numOfHistorySlices, unsigned int sliceTime);
  ~DSLModel();
  void Init(unsigned int currentTimeSlice=8, unsigned int numOfHistorySlices=10, unsigned int sliceTime=3000);
  void ReadModelFromFile(const std::string fileName);
  void AddObservations(int nodeHandle, unsigned long arrivalTime, int evidenceState);
  void Update();
  void UpdateTime(unsigned long currentTime);

  int GetId() {return id;}
  DSL_network* GetDSLNetwork() {return dslModel;}
  int GetCurrentSliceIndex(){return this->currentSliceIndex;}

private:

  unsigned int GenerateHash(const char * string, size_t len);
  void Test();                          //simple function for testing

  const char* CLASSNAME;
  int id;                      //hash code of the filename
  DSL_network* dslModel;                //bayesian network (smile lib)

  ObservationQueue obsQueue;
  int currentSliceIndex;                //Index of slice for t=0 (current evidence)

  //history time settings
  unsigned int numOfHistorySlices;
  unsigned int sliceTime;


};
}

#endif



