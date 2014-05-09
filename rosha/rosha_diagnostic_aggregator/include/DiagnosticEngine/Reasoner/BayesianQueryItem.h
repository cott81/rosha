/*
 * BayesianQueryItem.h
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

#ifndef BAYESIANQUERYITEM_H_
#define BAYESIANQUERYITEM_H_

#include <string>

namespace diagnostic_engine
{

class BayesianQueryItem
{
public:
  BayesianQueryItem();
  BayesianQueryItem(int nodeId, int stateId=0); //here to continue
  BayesianQueryItem(std::string nodeName, int nodeId, std::string stateName, int stateId);
  virtual ~BayesianQueryItem();

  std::string GetNodeName() const {return nodeName;}
  void SetNodeName(std::string nodeName) {this->nodeName = nodeName;}

  int GetNodeId() const {return nodeId; }
  void SetNodeId(int nodeId) { this->nodeId = nodeId; }

  int GetStateId() const {return stateId; }
  void SetStateId(int nodeId) { this->stateId = nodeId; }

  std::string GetStateName() const {return stateName;}
  void SetStateName(std::string stateName) {this->stateName = stateName;}

  double GetStateProp() const { return stateProp;}
  void SetStateProp(double stateProp) {this->stateProp = stateProp;}

private:
  std::string nodeName;
  int nodeId;
  std::string stateName;
  int stateId;
  double stateProp;
};

} /* namespace diagnostic_engine */
#endif /* BAYESIANQUERYITEM_H_ */
