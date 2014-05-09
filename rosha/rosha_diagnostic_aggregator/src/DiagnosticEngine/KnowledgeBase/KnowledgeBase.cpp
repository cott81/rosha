/*
 * KnowledgeBase.cpp
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
 *  Created on: Jan 31, 2013		1:44:47 PM
 *      Author: Dominik Kirchner
 */

#include "DiagnosticEngine/KnowledgeBase/KnowledgeBase.h"

using namespace diagnostic_engine;
using namespace std;

/*
KnowledgeBase::KnowledgeBase(ModelFactory* modelFactory) {

  cout << "KR: in const" << endl;
  this->CLASSNAME  = "KnowledgeBase";
  ROS_INFO("%s: in const", this->CLASSNAME);

}
*/

/*
void KnowledgeBase::RestructureModel() {
  ROS_INFO("%s : RestructureModel() not supported yet", this->CLASSNAME);
}
*/

long KnowledgeBase::AddModel(const string& fromFileName, bool useAsSystemModel, bool useModelsDefaultHistorySettingse) {

  ROS_INFO("%s: AddModel: not supported for this knowledge base.", this->CLASSNAME);
  return -1;
}

long KnowledgeBase::AddModel(const std::string& modelName, const string& fromFileName, bool useModelsDefaultHistorySettings) {

  ROS_INFO("%s: AddModel: not supported for this knowledge base.", this->CLASSNAME);
  return -1;
}


void KnowledgeBase::SetHistorySettings(unsigned int numOfHistorySlices,
                                unsigned int sliceTime,
                                unsigned int currentTimeSlice,
                                long modelId) {

  ROS_INFO("%s: SetHistorySettings: not supported for this knowledge base.", this->CLASSNAME);
  return;
}

void KnowledgeBase::UpdateTime(unsigned long currentTime) {

  ROS_WARN("%s: UpdateTime(unsigned long currentTime): not supported for this knowledge base.", this->CLASSNAME);
  return;
}


unsigned long KnowledgeBase::hash(const char *str) {
        //from http://www.cse.yorku.ca/~oz/hash.html
    unsigned long hash = 5381;
    int c;

    while (c = *str++) {
        hash = ((hash << 5) + hash) + c; /* hash * 33 + c */
    }

    return hash;
}

void KnowledgeBase::AddModelLink(BayesianML modelLink) {

  ROS_WARN("%s: AddModelLink(BayesianML modelLink): not supported for this knowledge base.", this->CLASSNAME);
  return;
}

