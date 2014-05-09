/*
 * BayesianKC.cpp
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
 *  Created on: Jan 31, 2013		4:42:38 PM
 *      Author: Dominik Kirchner
 */

#include "DiagnosticEngine/KnowledgeBase/BayesianKC.h"

using namespace diagnostic_engine;
using namespace std;

BayesianKC::BayesianKC() {

  this->CLASSNAME = "BayesianKC";
  this->partModelId = -1;
  this->nodeHandle = -1;
  this->nodeEvidenceState = -1;
  this->arrivalTime = -1;
}

void BayesianKC::Init(long partModelId, int nodeHandle, int state, unsigned long arrivalTime) {

  SetPartModelId(partModelId);
  SetNodeHandle(nodeHandle);
  SetNodeEvidenceState(state);
  SetArrivalTime(arrivalTime);
}

BayesianKC::~BayesianKC() {

}




