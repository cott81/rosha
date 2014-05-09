/*
 * ModelFactory.cpp
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
 *  Created on: Feb 8, 2013		8:04:42 AM
 *      Author: Dominik Kirchner
 */

#include <DiagnosticEngine/KnowledgeBase/ModelFactory.h>

using namespace diagnostic_engine;
using namespace std;


ModelFactory::ModelFactory() {

  this->CLASSNAME = "ModelFactory";
  this->modelFileName = "";
}

ModelFactory::~ModelFactory() {

}

void ModelFactory::SetHistorySettings(unsigned int numOfHistorySlices,
                                unsigned int sliceTime,
                                unsigned int currentTimeSlice) {

  ROS_INFO("%s: SetHistorySettings: not supported for this model.", this->CLASSNAME);
  return;
}




