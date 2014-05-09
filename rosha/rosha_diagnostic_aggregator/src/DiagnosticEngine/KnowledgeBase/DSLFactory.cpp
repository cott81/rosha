/*
 * DSLFactory.cpp
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
 *  Created on: Feb 8, 2013		8:25:55 AM
 *      Author: Dominik Kirchner
 */

#include <DiagnosticEngine/KnowledgeBase/DSLFactory.h>

using namespace diagnostic_engine;
using namespace std;

DSLFactory::DSLFactory() {

  this->CLASSNAME = "DSLFactory";
  //default
  this->currentTimeSlice = 8;
  this->numOfHistorySlices = 10;
  this->sliceTime = 3000;
}

DSLFactory::~DSLFactory() {

}

Model* DSLFactory::CreateModel() {

  DSLModel* dslModel = new DSLModel(this->currentTimeSlice, this->numOfHistorySlices, this->sliceTime);
  if (!this->modelFileName.empty()) {
    dslModel->ReadModelFromFile(this->modelFileName);
  } else {
    ROS_WARN("%s no model file given. Create a empty Model object", this->CLASSNAME);
  }

  return dslModel;
}

void DSLFactory::SetHistorySettings(unsigned int numOfHistorySlices,
                                unsigned int sliceTime,
                                unsigned int currentTimeSlice) {

  ROS_INFO("%s: SetHistorySettings: set numOfHistorySlices=%d, sliceTime=%d",
           this->CLASSNAME, numOfHistorySlices, sliceTime);
  this->numOfHistorySlices = numOfHistorySlices;
  this->sliceTime = sliceTime;
  this->currentTimeSlice = currentTimeSlice;
  return;
}



