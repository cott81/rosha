/*
 * DiagnosticEngine.cpp
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
 *  Created on: Jan 31, 2013		1:34:44 PM
 *      Author: Dominik Kirchner
 */


#include "DiagnosticEngine/DiagnosticEngine.h"

using namespace diagnostic_engine;
using namespace std;

/* This is the critical section object (statically allocated). */
//DiagnosticEngine::cs_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t cs_mutex = PTHREAD_MUTEX_INITIALIZER;
DiagnosticEngine* DiagnosticEngine::instance = 0;


DiagnosticEngine::DiagnosticEngine() {

  this->CLASSNAME = "DiagnosticEngine";
}

DiagnosticEngine::~DiagnosticEngine() {}


void DiagnosticEngine::destroy() {
   delete instance;
   instance = 0;
}

DiagnosticEngine* DiagnosticEngine::getInstance() {

  if (!instance) {
    pthread_mutex_lock( &cs_mutex );
    if (!instance) {
      instance = new DiagnosticEngine();
    }
    pthread_mutex_unlock( &cs_mutex );
  }
  return instance;
  //return *instance;
}


void DiagnosticEngine::Init(std::string modelFileName ) {

  ROS_INFO("%s: Init(): create bayesian knowledge base (BayesianKB), bayesian model factory (DSLFactory -> DSLModel), and bayesian reasoner (BayesianReasoner)", this->CLASSNAME);

  // create KnowledgeBase
  BayesianKB* bkb = new BayesianKB();
  DSLFactory* dslFactory = new DSLFactory();
//  dslFactory->SetModelFileName(modelFileName);
  dslFactory->SetModelFileName("");
  bkb->Init(dslFactory);
  this->knowledgeBase = bkb;

  /*
  std::string path = ros::package::getPath("my_diagnostic_aggregator");
  //TODO: replace hard coded path ... one central system model
//  path = path + "/model/SystemModel.xdsl";
  path = path + "/model/" + modelFileName;
  int ret = this->knowledgeBase->AddModel(path, true);
  */

  // create Reasoner
  this->reasoner = new BayesianReasoner();

  //create Classification
  return;
}


