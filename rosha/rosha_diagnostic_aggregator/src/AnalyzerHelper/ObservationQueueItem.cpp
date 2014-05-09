/*
 * ObservationQueueItem.cpp
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
 *  Created on: Jan 17, 2013		4:02:30 PM
 *      Author: Dominik Kirchner
 */


#include "AnalyzerHelper/ObservationQueueItem.h"

using namespace diagnostic_aggregator;

ObservationQueueItem::ObservationQueueItem(){
  this->delay = 0;
  this->correspondingTimeSlice = 0;
}

ObservationQueueItem::ObservationQueueItem(unsigned long arrivalTime, int evidenceState) {
  this->delay = 0;
  this->correspondingTimeSlice = 0;

  this->arrivalTime = arrivalTime;
  this->evidenceState = evidenceState;


}

ObservationQueueItem::~ObservationQueueItem() {

}

