/*
 * BayesianKC.h
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
 *  Created on: Jan 31, 2013		4:39:41 PM
 *      Author: Dominik Kirchner
 */

#ifndef BAYESIANKC_H_
#define BAYESIANKC_H_

#include <DiagnosticEngine/KnowledgeBase/KnowledgeChange.h>

namespace diagnostic_engine {

class BayesianKC : public KnowledgeChange {

public:
  BayesianKC();
  ~BayesianKC();
  void Init(long partModelId, int nodeHandle, int state, unsigned long arrivalTime);

  void SetPartModelId (long id) {this->partModelId = id;}
  long GetPartModelId() {return this->partModelId;}

  void SetNodeHandle(int handle) {this->nodeHandle = handle;}
  int GetNodeHandle() {return this->nodeHandle;}

  void SetNodeEvidenceState(int state){this->nodeEvidenceState = state;}
  int GetNodeEvidenceState() {return this->nodeEvidenceState;}

  void SetArrivalTime(unsigned long arrivalTime){this->arrivalTime = arrivalTime;}
  unsigned long GetArrivalTime() {return this->arrivalTime;}

private:
  //data needed for updating the net
  long partModelId;
  int nodeHandle;
  int nodeEvidenceState;
  unsigned long arrivalTime;

  const char* CLASSNAME;
};
}



#endif /* BAYESIANKC_H_ */
