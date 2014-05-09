/*
 * DSLFactory.h
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
 *  Created on: Feb 8, 2013		8:24:14 AM
 *      Author: Dominik Kirchner
 */

#ifndef DSLFACTORY_H_
#define DSLFACTORY_H_

#include <DiagnosticEngine/KnowledgeBase/ModelFactory.h>
#include <DiagnosticEngine/KnowledgeBase/DSLModel.h>

namespace diagnostic_engine {

class DSLFactory : public ModelFactory {

public:
  DSLFactory();
  virtual ~DSLFactory();
  virtual Model* CreateModel();
  virtual void SetHistorySettings(unsigned int numOfHistorySlices,
                                  unsigned int sliceTime,
                                  unsigned int currentTimeSlice);


private:

  const char* CLASSNAME;

  //history time settings
  unsigned int currentTimeSlice;
  unsigned int numOfHistorySlices;
  unsigned int sliceTime;               //in ms

};
}



#endif /* DSLFACTORY_H_ */
