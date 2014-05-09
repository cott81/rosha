/*
 * BayesianQueryItem.cpp
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
 *  Created on: Mar 5, 2013		11:28:34 AM
 *      Author: Dominik Kirchner
 */

#include <DiagnosticEngine/Reasoner/BayesianQueryItem.h>

using namespace std;

namespace diagnostic_engine
{

BayesianQueryItem::BayesianQueryItem()
{
  this->nodeName = "unknown";
  this->nodeId = 0;
  this->stateId = 0;
  this->stateName = "unspecified";
  this->stateProp = -1.0;
}

BayesianQueryItem::BayesianQueryItem(int nodeId, int stateId) {

  this->SetNodeId(nodeId);
  this->SetStateId(stateId);
}

BayesianQueryItem::BayesianQueryItem(string nodeName, int nodeId, string stateName, int stateId) {

  this->SetNodeName(nodeName);
  this->SetNodeId(nodeId);
  this->SetStateName(stateName);
  this->SetStateId(stateId);
}

BayesianQueryItem::~BayesianQueryItem()
{
  // TODO Auto-generated destructor stub
}

} /* namespace diagnostic_engine */
