/*
 * BayesianML.h
 *
 * Copyright 2014 Carpe Noctem, Distributed Systems Group,
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
 *  Created on: Mar 4, 2014		1:43:36 PM
 *      Author: dominik
 */

#ifndef BAYESIANML_H_
#define BAYESIANML_H_

#include <string>
#include <vector>

#include <ros/ros.h>

using namespace std;

namespace diagnostic_engine
{

class BayesianML
{
public:
  BayesianML();
  BayesianML(long compModelId, int compModelNodeId, int systemModelNodeId);
  virtual ~BayesianML();

  int GetCompModelNodeId() const
  {
    return compModelNodeId;
  }

  void SetCompModelNodeId(int compModelNodeId)
  {
    this->compModelNodeId = compModelNodeId;
  }

  int GetSystemModelNodeId() const
  {
    return systemModelNodeId;
  }

  void SetSystemModelNodeId(int systemModelNodeId)
  {
    this->systemModelNodeId = systemModelNodeId;
  }

  long GetModelId() const
  {
    return modelId;
  }

  void SetModelId(long modelId)
  {
    this->modelId = modelId;
  }

  const std::string& GetCompModelNodeName() const
  {
    return compModelNodeName;
  }

  void SetCompModelNodeName(const std::string& compModelNodeName)
  {
    this->compModelNodeName = compModelNodeName;
  }

  const std::string& GetSystemModelNodeName() const
  {
    return systemModelNodeName;
  }

  void SetSystemModelNodeName(const std::string& systemModelNodeName)
  {
    this->systemModelNodeName = systemModelNodeName;
  }

  const std::vector<unsigned short>& GetStateMapping() const
  {
    return stateMapping;
  }

  void SetStateMapping(const std::vector<unsigned short>& stateMapping)
  {
    //check if the vector conatains only correct numbers (max length, no doublicates)
    bool error = true;
    for (unsigned short numberToCHeck=0; numberToCHeck<stateMapping.size(); numberToCHeck++) {

      for (unsigned short ii=0; ii<stateMapping.size(); ii++) {

        if (stateMapping[ii] == numberToCHeck){
          error = false;
          break;
        }
      }

      if (error == true) {
        // error flag not reset in loop
        ROS_ERROR("%s: SetStateMapping(): the given vector does not contain a correct sequence of numbers between 0 and max. size. %d number is missing",
                  this->CLASSNAME, numberToCHeck);
        return;
      }
      //reset flag
      error = true;
    }

    this->stateMapping = stateMapping;
  }

  void SetDirectMapping(unsigned short numStates) {

    for (unsigned short i=0; i<numStates; i++) {
      this->stateMapping.push_back(i);
    }
  }

private:
  const char* CLASSNAME;

  int systemModelNodeId;
  std::string systemModelNodeName;

  int compModelNodeId;
  std::string compModelNodeName;

  //TODO: stateMapping ... what compModelState is set as waht systemModelState
//  [1, 3, 2] -> [1,2,3]        assume equally legthed states!
  std::vector<unsigned short> stateMapping;     /*!< component model node state to system model node state. E.g. [1, 3, 2] means comp state 1 is mapped to system mode state 1, comp state 3 to sys state 2 */



  long modelId;

};

} /* namespace diagnostic_engine */
#endif /* BAYESIANML_H_ */
