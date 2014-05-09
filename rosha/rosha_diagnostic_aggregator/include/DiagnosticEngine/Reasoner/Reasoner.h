/*
 * Reasoner.h
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
 *  Created on: Jan 31, 2013		1:49:12 PM
 *      Author: Dominik Kirchner
 */

#ifndef REASONER_H_
#define REASONER_H_

//elements for visiting (type whereever the model structure is defined)
#include <DiagnosticEngine/KnowledgeBase/DSLModel.h>

//#include <DiagnosticEngine/KnowledgeBase/BayesianKB.h>  //PROBLEM ... includes ... is included from KnowledgeBase



#include "ros/ros.h"

namespace diagnostic_engine {

class BayesianKB;       //forward declaration

class Reasoner {

public:
  Reasoner();
  virtual ~Reasoner();

  //interface
  //virtual void Infer() = 0;
  virtual void Visit(DSLModel& model) = 0;
  //virtual void Visit(BayesianKB bayesianKB) = 0;


private:

  const char* CLASSNAME;


};
}

#endif /* REASONER_H_ */
