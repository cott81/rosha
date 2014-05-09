/*
 * ObservationQueue.h
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
 *  Created on: Jan 17, 2013		3:57:44 PM
 *      Author: Dominik Kirchner
 */

#ifndef OBSERVATIONQUEUE_H_
#define OBSERVATIONQUEUE_H_


#include "AnalyzerHelper/ObservationQueueItem.h"

#include <ros/ros.h>
#include <iostream>
#include <vector>

namespace diagnostic_aggregator {

//passive observer type enum
static const unsigned short sizeObsType = 3;
enum ObsType {
  CPU_OBS=0,
  MEM_OBS,
  THREAD_OBS
};

class ObservationQueue {

public:
  ObservationQueue();
  void Init(unsigned int numOfSlices, unsigned int sliceTime);
  ~ObservationQueue();
  //better data structure map, because use nodeHandles as keys, no predefined OBS needed
  void AddItem(int nodeHandle, unsigned long arrivalTime, int evidenceState);
  void AddItem(int nodeHandle, ObservationQueueItem item);
  void PrintItems();
  void UpdateRelativeDelay(unsigned int arrival);

  std::map<int, std::vector<ObservationQueueItem> > observations;        //nodeHandle (nodeId), vector od Observations

private:

  void OrderedAdd(int nodeHandle, ObservationQueueItem item);


  const char* CLASSNAME;
  unsigned int numOfSlices;
  unsigned int sliceTime;       //in ms
  unsigned int dbnHistoryLentgh; //in ms
  unsigned long newestArrivalTime;      //newest arrival time independend of obsType

};
}

#endif /* OBSERVATIONQUEUE_H_ */
