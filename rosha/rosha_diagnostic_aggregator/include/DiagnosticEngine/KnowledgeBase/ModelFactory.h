/*
 * ModelFactory.h
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
 *  Created on: Feb 8, 2013		8:05:32 AM
 *      Author: Dominik Kirchner
 */

#ifndef MODELFACTORY_H_
#define MODELFACTORY_H_

#include <DiagnosticEngine/KnowledgeBase/Model.h>
#include <string>
#include <ros/ros.h>

namespace diagnostic_engine {

class ModelFactory {

public:
  ModelFactory();
  virtual ~ModelFactory();
  virtual Model* CreateModel() = 0;
  //virtual Model* CreateModel(const std::string fromFile) = 0;
  virtual void SetModelFileName(const std::string fromFile) {this->modelFileName = fromFile;}
  virtual void SetHistorySettings(unsigned int numOfHistorySlices,
                                  unsigned int sliceTime,
                                  unsigned int currentTimeSlice);


protected:
  std::string modelFileName;

private:
  const char* CLASSNAME;

};
}

#endif /* MODELFACTORY_H_ */
