/*
 * ObservationQueue.cpp
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
 *  Created on: Jan 17, 2013		3:57:18 PM
 *      Author: Dominik Kirchner
 */

#include "AnalyzerHelper/ObservationQueue.h"
#include "AnalyzerHelper/ObservationQueueItem.h"

using namespace diagnostic_aggregator;
using namespace std;

ObservationQueue::ObservationQueue() {

  this->CLASSNAME = "ObservationQueue";
}

void ObservationQueue::Init(unsigned int numOfHistorySlices, unsigned int sliceTime) {

  //porpose init: decide the time to initizalize vars

  this->numOfSlices = numOfHistorySlices;
  this->sliceTime = sliceTime;
  this->dbnHistoryLentgh = numOfSlices * sliceTime;       //in ms
  this->newestArrivalTime= 0;

  //init vectors for all ... registered observers???
  /*
  for (int i=0; i<sizeObsType; i++ ) {
    this->observations.push_back(std::vector<ObservationQueueItem>(0) );
  }
  */

  /*
  //for TESTING the list update ...


  //add items in strict order ... because time causality
  ObservationQueueItem item (20000, 1);
  AddItem(CPU_OBS, item);

  ObservationQueueItem currentItem333 (500, 30);
    AddItem(MEM_OBS, currentItem333);
  ObservationQueueItem item22 (2500, 10);
    AddItem(MEM_OBS, item22);
  ObservationQueueItem currentItem2 (6000, 20);
    AddItem(MEM_OBS, currentItem2);
  ObservationQueueItem currentItem33 (1000, 30);
    AddItem(MEM_OBS, currentItem33);


  ObservationQueueItem item2 (1200, 2);
  AddItem(CPU_OBS, item2);
  ObservationQueueItem currentItem (4000, 3);
  AddItem(CPU_OBS, currentItem);
  ObservationQueueItem itemxy (2000, 3);
  AddItem(CPU_OBS, itemxy);

  ObservationQueueItem itemx (3000, 3);
  AddItem(CPU_OBS, itemx);


  PrintItems();
*/

}

ObservationQueue::~ObservationQueue() {
}


void ObservationQueue::AddItem(int nodeHandle,  unsigned long arrivalTime, int evidenceState) {

  ObservationQueueItem item (arrivalTime, evidenceState);
  AddItem(nodeHandle, item);

  //for debugging only
  //PrintItems();

}

void ObservationQueue::AddItem(int nodeHandle, ObservationQueueItem item) {

  //ROS_INFO("ObservationQueue::AddItem: Node Handle: %d, evidenceState item: %d, arrival time: %d",
  //         nodeHandle, item.evidenceState, item.arrivalTime);

  //check causal order (comm delays ...)
  //if arrival time < oldArrivalTime ... independend of obsType -> ordered insert in list
  // !this->observations[obsType].empty() &&

  //node not in map (key not found) -> create entry for this node
  if (this->observations.find(nodeHandle) == this->observations.end()) {
    ROS_DEBUG("%s: AddItem: key %d not found, create new entry for that key", this->CLASSNAME, nodeHandle);
    vector<ObservationQueueItem> nodeObsVector(0);
    this->observations.insert(std::pair<int, vector<ObservationQueueItem> >(nodeHandle, nodeObsVector) );
  }

  if (item.arrivalTime < this->newestArrivalTime) {
    ROS_WARN("Temporal causality violation.");
    //simply call OrderedAdd in all violation cases (violation to own list, other list, both ...)
    OrderedAdd(nodeHandle, item);
  } else {
    this->observations[nodeHandle].push_back(item);
    this->newestArrivalTime = this->observations[nodeHandle].back().arrivalTime;
  }

  //calculate the delay of the rest of elements to the new one
  // could/should be called once for an update
  UpdateRelativeDelay(this->newestArrivalTime);

  //TODO: speed it up ... if causality violation
  // -> no newestArrival time -> update only list of current observer type
}

void ObservationQueue::OrderedAdd(int nodeHandle, ObservationQueueItem item) {

  //iterate over old list (without current item) beginning from the end and
  //place item
  //no "better" ordering because violations are more probable to occur at the end
  for (int i=this->observations[nodeHandle].size()-1; i>=0; i--) {
    if (this->observations[nodeHandle][i].arrivalTime > item.arrivalTime) {
      continue;
    } else {
      //insert after i
      this->observations[nodeHandle].insert(this->observations[nodeHandle].begin()+i+1, item);
      return;
    }
  }

  //item.arrival < all items in list -> insert at first
  this->observations[nodeHandle].insert(this->observations[nodeHandle].begin(), item);
  return;

}

void ObservationQueue::PrintItems(){

  //iterate over map
  for (map<int, vector<ObservationQueueItem> >::iterator iter=this->observations.begin();
      iter != this->observations.end(); ++iter) {
    cout << "\n\nObserver for node: " << (*iter).first << endl;
    for (int i=0; i<this->observations[(*iter).first].size(); i++) {
      cout << "\nItem: " << i << endl;
      cout << "\t arrivalTime: " << this->observations[(*iter).first][i].arrivalTime << endl;
      cout << "\t delay to current item: " << this->observations[(*iter).first][i].delay << endl;
      cout << "\t time slice: " << this->observations[(*iter).first][i].correspondingTimeSlice << endl;
      cout << "\t evidenceState: " << this->observations[(*iter).first][i].evidenceState << endl;
    }
  }
}

void ObservationQueue::UpdateRelativeDelay(unsigned int arrival) {

  ROS_DEBUG("%s UpdateRelativeDelay(unsigned int arrival): arrival time: %d [ms]",
            this->CLASSNAME, arrival);

  unsigned int timeSlice = 0;
  unsigned int r=0;
  unsigned int limit = this->sliceTime/2;

  //for all map items <nodeId, obs lists>
  for (map<int, vector<ObservationQueueItem> >::iterator iter=this->observations.begin();
      iter != this->observations.end();
      ++iter)
  {

    //only no element-> nothing to be done
    if (this->observations[(*iter).first].size() < 1) {
      continue;
    }

    //iterate over all items (but the last one: delay is 0 -> not true any more)
    // include last one, because have to update all list of obs, with a new evidence or not
    for (int i=this->observations[(*iter).first].size()-1; i>=0; i--) {

      //if (negative) -> causal order violation ->
        this->observations[(*iter).first][i].delay = arrival - this->observations[(*iter).first][i].arrivalTime;

        //set corresponding timeSlice: rounding
        timeSlice = this->observations[(*iter).first][i].delay / this->sliceTime;
        r = this->observations[(*iter).first][i].delay % this->sliceTime;    //in intervall [0, sliceTime]

        if (r < limit) {
          this->observations[(*iter).first][i].correspondingTimeSlice = timeSlice;
        } else {
          this->observations[(*iter).first][i].correspondingTimeSlice = timeSlice +1;
        }

        //check if history exceeded
        if (this->observations[(*iter).first][i].correspondingTimeSlice > this->numOfSlices) {
          //remove this item and all further items, assume a timely ordered list
          this->observations[(*iter).first].erase(this->observations[(*iter).first].begin(), this->observations[(*iter).first].begin()+i+1); //last item exclusive
          break;
        }

        //reminder: ... rounding: more elements on one timeSlice: choose the most current item to use as evidence
        // -> update dbn from beginning of the list ... overwrite older elements on the time slice
        // -> better algorithm?

    }

    //PrintItems();
  }

}






