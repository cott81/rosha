/*
 * BayesianML.cpp
 *
 * Copyright 2014 Carpe Noctem, Distributed Systems Group,
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
 *  Created on: Mar 4, 2014		1:43:36 PM
 *      Author: dominik
 */

#include "DiagnosticEngine/KnowledgeBase/BayesianML.h"

namespace diagnostic_engine
{

BayesianML::BayesianML()
{
  this->CLASSNAME = "BayesianML";

  this->systemModelNodeId = -1;
  this->systemModelNodeName = "";

  this->compModelNodeId = -1;
  this->compModelNodeName = "";

  this->modelId = -1;
}

BayesianML::BayesianML(long compModelId, int compModelNodeId, int systemModelNodeId){

  this->CLASSNAME = "BayesianML";
  SetModelId(compModelId);
  SetCompModelNodeId(compModelNodeId);
  SetSystemModelNodeId(systemModelNodeId);
}

BayesianML::~BayesianML()
{
}

} /* namespace diagnostic_engine */
