/*
 * BayesianReasoner.h
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
 *  Created on: Feb 5, 2013		6:04:37 PM
 *      Author: Dominik Kirchner
 */

#ifndef BAYESIANREASONER_H_
#define BAYESIANREASONER_H_

#include <DiagnosticEngine/KnowledgeBase/DSLModel.h>
#include <DiagnosticEngine/KnowledgeBase/BayesianKB.h>
#include <DiagnosticEngine/Reasoner/Reasoner.h>
#include <DiagnosticEngine/Reasoner/BayesianQueryItem.h>

namespace diagnostic_engine {

class BayesianReasoner : public Reasoner {

public:
  BayesianReasoner();
  ~BayesianReasoner();

  //visitor methods
  void Visit(DSLModel &model);
  //void Visit(BayesianKB bayesianKB);
  void SetQuery(std::vector<int>* nodeHandles);
  void SetQuery(std::map<int, int>* nodeHandleStates);
  void SetQuery(std::vector<BayesianQueryItem>* query);
  std::vector<BayesianQueryItem>* GetResults();

private:

  const char* CLASSNAME;

//  std::map<int, double>* querySet;
  //vector of query items ... because it coulf be possible to query multiple states for one key/nodehandle!
  std::vector<BayesianQueryItem>* querySet;

};
}




#endif /* BAYESIANREASONER_H_ */
