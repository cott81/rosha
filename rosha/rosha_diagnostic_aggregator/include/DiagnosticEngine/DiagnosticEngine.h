/*
 * DiagnosticEngine.h
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
 *  Created on: Jan 31, 2013		1:35:10 PM
 *      Author: Dominik Kirchner
 */

#ifndef DIAGNOSTICENGINE_H_
#define DIAGNOSTICENGINE_H_


#include <DiagnosticEngine/KnowledgeBase/BayesianKB.h>          //??? needed: <same>
#include <DiagnosticEngine/Reasoner/BayesianReasoner.h>         //??? needed: because you create a specific object for the knowledgebase and reasoner
#include <DiagnosticEngine/KnowledgeBase/DSLFactory.h>

#include <DiagnosticEngine/KnowledgeBase/KnowledgeBase.h>
#include <DiagnosticEngine/Reasoner/Reasoner.h>
#include <DiagnosticEngine/KnowledgeBase/ModelFactory.h>

#include <pthread.h>
#include <ros/ros.h>
#include <ros/package.h>

namespace diagnostic_engine {

class DiagnosticEngine {

  //here singelton, because problems for a singelton in an interface (abstract class)
  // use additional inheritance in plugins to provide some service functions

private:
  static DiagnosticEngine* instance;
  DiagnosticEngine();
  ~DiagnosticEngine();

  const char* CLASSNAME;

  KnowledgeBase* knowledgeBase;
  Reasoner* reasoner;

public:
  static DiagnosticEngine* getInstance();
  static void destroy();

  void Init(std::string modelFileName = "");
  KnowledgeBase* GetKnowledgeBase() {return this->knowledgeBase;}
  Reasoner* GetReasoner() {return this->reasoner;}

};
}


#endif /* DIAGNOSTICENGINE_H_ */
