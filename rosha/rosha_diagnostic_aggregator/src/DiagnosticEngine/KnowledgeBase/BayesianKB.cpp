/*
 * BayesianKB.cpp
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
 *  Created on: Jan 31, 2013		3:28:39 PM
 *      Author: Dominik Kirchner
 */

#include "DiagnosticEngine/KnowledgeBase/BayesianKB.h"

using namespace diagnostic_engine;
using namespace std;

BayesianKB::BayesianKB () {
  this->CLASSNAME = "BayesianKB";
}

BayesianKB::~BayesianKB() {

}

void BayesianKB::Init(ModelFactory* modelFactory) {

  ROS_INFO("%s: Init", this->CLASSNAME);
  this->compModels;
  this->bayesianCMCount = 0;

  this->modelFactory = modelFactory;            //for later use? model change etc ... simply recall of Init?
  this->model = modelFactory->CreateModel();

  //history time settings
  this->numOfHistorySlices = 10;
  this->sliceTime = 3;

  this->systemModel = NULL;
}

void BayesianKB::SetHistorySettings(unsigned int numOfHistorySlices,
                                unsigned int sliceTime,
                                unsigned int currentTimeSlice,
                                long modelId) {

  this->numOfHistorySlices = numOfHistorySlices;
  this->sliceTime = sliceTime;
  this->currentTimeSlice = currentTimeSlice;

  ROS_INFO("%s: SetHistorySettings: numOfHistorySlices = %u, sliceTime = %u, for modelId = %ld",
           this->CLASSNAME, numOfHistorySlices, sliceTime, modelId);

  if (modelId != -1) {
    ROS_WARN("%s: SetHistorySettings: history settings for individual models are currently not supported!", this->CLASSNAME);
  }

  ROS_WARN("%s: SetHistorySettings: no yet fully supported!", this->CLASSNAME);
}


long BayesianKB::AddModel(const string& fromFileName, bool useAsSystemModel, bool useModelsDefaultHistorySettings) {

  ROS_INFO("%s : AddModel: from filename: %s", this->CLASSNAME, fromFileName.c_str());

  this->modelFactory->SetModelFileName(fromFileName);
  if (!useModelsDefaultHistorySettings) {
    this->modelFactory->SetHistorySettings(this->numOfHistorySlices, this->sliceTime, this->currentTimeSlice); //what of slice of the current time?
  }
  Model* m = this->modelFactory->CreateModel();

  DSLModel* bayesianCM = dynamic_cast<DSLModel*>(m);
  if (bayesianCM == NULL) {
    ROS_ERROR("%s: AddModel: cast error.", this->CLASSNAME);
  }

  int keyToCheck = -1;
  if (useAsSystemModel) {

    // set model as system model
    this->systemModel = bayesianCM;

  } else {

    //set model as component model and store it in map and check double entries

    //check if key already present
    keyToCheck = bayesianCM->GetId();
    bool keyExists = true;
    while (keyExists) {
      if (compModels.find(keyToCheck) == compModels.end()){
        //not found
        ROS_DEBUG("%s: AddModel: add key %d", this->CLASSNAME, keyToCheck);
        this->compModels.insert(std::pair<long, DSLModel*>(keyToCheck, bayesianCM));
        keyExists = false;
        break;
      } else {
        //found
        ROS_DEBUG("%s: AddModel: key %d already in map. Increase key and try again.", this->CLASSNAME, keyToCheck);
        keyToCheck++;     //only negative ids?
      }
    }

    //not perfect to set it in this way
    SetCurrentModel(keyToCheck);
  }

  return keyToCheck;
}

long BayesianKB::AddModel(const std::string& modelName, const string& fromFileName, bool useModelsDefaultHistorySettings) {

  ROS_INFO("%s : AddModel: model name: %s from filename: %s", this->CLASSNAME, modelName.c_str(), fromFileName.c_str());

  this->modelFactory->SetModelFileName(fromFileName);
  if (!useModelsDefaultHistorySettings) {
    this->modelFactory->SetHistorySettings(this->numOfHistorySlices, this->sliceTime, this->currentTimeSlice); //what of slice of the current time?
  }
  Model* m = this->modelFactory->CreateModel();

  DSLModel* bayesianCM = dynamic_cast<DSLModel*>(m);
  if (bayesianCM == NULL) {
    ROS_ERROR("%s: AddModel: cast error.", this->CLASSNAME);
  }

  //check if key already present
  unsigned short counter = 1;
  stringstream ss;
  string modelNameExtended = modelName;
  long keyToCheck = hash(modelNameExtended.c_str());    //TODO: unsigned long -> long !!! fix this

  bool keyExists = true;
  while (keyExists) {
    if (compModels.find(keyToCheck) == compModels.end()){
      //not found
      ROS_DEBUG("%s: AddModel: add key %ld", this->CLASSNAME, keyToCheck);
      this->compModels.insert(std::pair<long, DSLModel*>(keyToCheck, bayesianCM));
      keyExists = false;
      break;
    } else {
      //found

      ss << modelName << "_" << counter;
      ss >> modelNameExtended;
      counter++;

      ROS_DEBUG("%s: AddModel: key %ld already exists in map. Extend model name to %s, recalculate hash, and try again.",
                this->CLASSNAME, keyToCheck, modelNameExtended.c_str());

      keyToCheck = hash(modelNameExtended.c_str());     //only negative ids?

    }
  }

  //not perfect to set it in this way
  SetCurrentModel(keyToCheck);

  return keyToCheck;
}

void BayesianKB::Update(std::vector<KnowledgeChange*> kcs) {

  //ROS_INFO("%s: Update: update knowledge base with vector of knowledge changes (evidences)", this->CLASSNAME);

  //1 include/add observations in part model (to ObservationQueue)

  //managed list with involved compModels for these konwledge changes
  bool nextKC;
  std::vector<long>involvedCompModelIds;

  //for all items in KnowledgeChange (list) add Observations (for all compModels)
  for (int i=0; i< kcs.size(); i++) {

    BayesianKC* bkc = dynamic_cast<BayesianKC*> ( kcs[i]);
    //no cast needed if the used functions would have been declared (virtual) in base class ...
    if (bkc == NULL) {
      ROS_ERROR("%s: Update(): downward cast error of KnowledgeChanges, Skip that Observation.",
                this->CLASSNAME);
      continue;
    }

    //check if the model of a konwledge change is already in the list involvedCompModelIds
    if (find(involvedCompModelIds.begin(), involvedCompModelIds.end(), bkc->GetPartModelId()) != involvedCompModelIds.end()) {
      //modelId already in list
    } else {
      involvedCompModelIds.push_back(bkc->GetPartModelId());
    }

    //check if the knowledge base knows the model at all
    if (this->compModels.find(bkc->GetPartModelId()) == this->compModels.end() ) {
      ROS_ERROR("%s: Update: comp model key not found in map. Skip that Observation.",
                this->CLASSNAME);
      continue;
    } else {
      this->compModels[bkc->GetPartModelId()]->AddObservations(bkc->GetNodeHandle(),
                                                           bkc->GetArrivalTime(),
                                                           bkc->GetNodeEvidenceState());
    }
  }

  //2 update net ... set (ordered) observations to net ... for all compModels (analyzer matches more comps)
  //update all compModels??? Only compModels with new observations!
  for (int i = 0; i<involvedCompModelIds.size(); i++) {
//    ROS_INFO("%s: Update(): DEBUG: update all involved models: count: %d", this->CLASSNAME, i);
    this->compModels[involvedCompModelIds[i]]->Update();
  }

}

//Update the current time for all models in the knowledge base
void BayesianKB::UpdateTime(unsigned long currentTime) {

  DSLModel* bcm = dynamic_cast<DSLModel*>(this->model);

  if (bcm == NULL) {
    ROS_ERROR("%s: UpdateTime(unsigned long currentTime): downward cast error of Model, Skip that update.",
              this->CLASSNAME);
    return;
  }

  ROS_DEBUG("%s UpdateTime: before call of dsl model update", this->CLASSNAME);
  //call the Update function of the CURRENT model
  bcm->UpdateTime(currentTime);
}

void BayesianKB::UpdateTime(unsigned long currentTime, int modelId) {

  //check the model id exists
  if (this->compModels.find(modelId) == this->compModels.end()) {
    //not found
    ROS_ERROR("%s: UpdateTime(unsigned long currentTime, int modelId): model id not found as a compModel in the knowledge base", this->CLASSNAME);
    return;
  }

  //call the Update function of the model
  this->compModels[modelId]->UpdateTime(currentTime);
}

Model* BayesianKB::GetModel(long modelId, bool useSystemModel) {

  DSLModel* bcm;
  if (useSystemModel) {
    bcm = this->systemModel;
  } else {
    bcm = this->compModels[modelId];
  }
  return bcm;
}


void BayesianKB::Accept(Reasoner &visitor) {

  //TODO: thread safe ...
  //lock ... ???
  mutexObj.lock();

  DSLModel* bcm = dynamic_cast<DSLModel*>(this->model);


  if (bcm == NULL) {
    ROS_ERROR("%s: Accept(): downward cast error of Model, Skip that reasoning.",
              this->CLASSNAME);
    return;
  }
  visitor.Visit(*bcm);

  if (this->systemModel != NULL) {
    //TODO: take the target node and update the system model
    //long modelId = bcm->GetId();
    UpdateSystemModel(bcm, this->currentModelId);
//    InferSystemModel();
  }

  mutexObj.unlock();

}


void BayesianKB::SetCurrentModel(long modelId, bool useSystemModel) {

  if (useSystemModel) {
    this->model = this->systemModel;
    //id collision between hash of component name -> id and the original dsl id of
    // the system model possible.
    this->currentModelId = this->systemModel->GetId();
  } else {
    this->model = this->compModels[modelId];
    this->currentModelId = modelId;
  }
}

/*
void BayesianKB::RestructureModel() {

  ROS_INFO("not implemented");
}
*/

/*
void BayesianKB::SetSystemModel(const string & fromFileName, bool useModelsDefaultHistorySettings=true) {

  this->modelFactory->SetModelFileName(fromFileName);
  if (!useModelsDefaultHistorySettings) {
    this->modelFactory->SetHistorySettings(this->numOfHistorySlices, this->sliceTime, this->currentTimeSlice); //what of slice of the current time?
  }
  Model* m = this->modelFactory->CreateModel();

  this->systemModel = dynamic_cast<DSLModel*>(m);
  if (this->systemModel == NULL) {
    ROS_ERROR("%s: SetSystemModel: cast error.", this->CLASSNAME);
  }

}
*/

void BayesianKB::AddModelLink(BayesianML modelLink) {

  // check if there is a system model present at all
  if (this->systemModel == NULL) {
    ROS_ERROR("%s: AddModelLink(): system model not defined. Can not register any link.", this->CLASSNAME);
    return;
  }

  //if there is no name of both specified ... search for the name

  //check and complete the comp model infos
  if(!modelLink.GetCompModelNodeName().empty()) {
    //we have a name
    int compNodeId = this->compModels[modelLink.GetModelId()]->GetDSLNetwork()->FindNode(modelLink.GetCompModelNodeName().c_str());
    //check if inconsistent data was set
    if (modelLink.GetCompModelNodeId() != compNodeId) {
      ROS_DEBUG("%s: AddModelLink(): comp model node name %s does not fit the given comp model node id %d. We reset the id to %d!",
               this->CLASSNAME, modelLink.GetCompModelNodeName().c_str(), modelLink.GetCompModelNodeId(), compNodeId);
      //set the id
      modelLink.SetCompModelNodeId(compNodeId);
    }

  } else if (modelLink.GetCompModelNodeId() > -1 ) {
    //we have an id
    const char* name = this->compModels[modelLink.GetModelId()]->GetDSLNetwork()->GetNode(modelLink.GetCompModelNodeId())->GetId();
    // check if inconsistent data was set
    if (modelLink.GetCompModelNodeName() != name ) {
      ROS_DEBUG("%s: AddModelLink(): comp model node id %d does not fit the given comp model name %s. We reset the name to %s!",
               this->CLASSNAME,  modelLink.GetCompModelNodeId(),  modelLink.GetCompModelNodeName().c_str(), name);
      //set the name
      modelLink.SetCompModelNodeName(name);
    }

  } else {
    ROS_ERROR("%s: AddModelLink(): it is neither a comp model node id nor a name specified. Skip the link.", this->CLASSNAME);
    return;
  }


  //check and complete the system model infos
  if ( (!modelLink.GetSystemModelNodeName().empty()) ) {

    //given system model ...
    const char * systemNodeName = modelLink.GetSystemModelNodeName().c_str();
    int systemModelNode = this->systemModel->GetDSLNetwork()->FindNode(systemNodeName);

    // node by name found?
    if (systemModelNode != DSL_OUT_OF_RANGE) {

      // check if inconsistent data was set
      if (modelLink.GetSystemModelNodeId() != systemModelNode ) {
        ROS_DEBUG("%s: AddModelLink(): system model node name %s does not fit the given system model id %d. We reset the id to %d!",
                 this->CLASSNAME,  systemNodeName,  modelLink.GetSystemModelNodeId(), systemModelNode);
        //set the id
        modelLink.SetSystemModelNodeId(systemModelNode);
      }

    } else {
      ROS_ERROR("%s: AddModelLink(): no system node found with the specified name: %s", this->CLASSNAME, modelLink.GetSystemModelNodeName().c_str());
      return;
    }

    //no system name, but system node id
  } else if (modelLink.GetSystemModelNodeId() > -1) {


//    const char* systemNodeName = this->systemModel->GetDSLNetwork()->GetNode(modelLink.GetSystemModelNodeId())->GetId();

    const char* systemNodeName;
    DSL_node *dslNode = this->systemModel->GetDSLNetwork()->GetNode(modelLink.GetSystemModelNodeId());
    if (dslNode != NULL) {
      systemNodeName = dslNode->GetId();
    } else {
      ROS_ERROR("%s: AddModelLink(): given system model node id %d seems to be incorrect. Skip this link.", this->CLASSNAME, modelLink.GetSystemModelNodeId());
      return;
    }

    //is empty for sure!
    modelLink.SetSystemModelNodeName(systemNodeName);

    //no system node or system node id  -> use comp node name
  } else {

    // find node with identical name
    const char* name = modelLink.GetCompModelNodeName().c_str();
    int systemModelNode = this->systemModel->GetDSLNetwork()->FindNode(name);
    ROS_INFO("%s: AddModelLink(): id of the system model node: %d", this->CLASSNAME, systemModelNode);

    // node by name found?
    if (systemModelNode != DSL_OUT_OF_RANGE) {
      modelLink.SetSystemModelNodeName(name);
      modelLink.SetSystemModelNodeId(systemModelNode);
    } else {
      ROS_ERROR("%s: AddModelLink(): no system node found with the specified name: %s", this->CLASSNAME, name);
      return;
    }

  }


  //check if the num of states of the system node is equal to the comp node
  if (!modelLink.GetStateMapping().empty()) {

    //check if the size matches

//    working code: better for DBN?
//    DSL_Dmatrix* matrix =this->systemModel->GetDSLNetwork()->GetNode(modelLink.GetSystemModelNodeId())->Value()->GetMatrix();
//    int numOfSystemStates = matrix->GetSizeOfDimension(0);

    int numOfSystemStates = this->systemModel->GetDSLNetwork()->GetNode(modelLink.GetSystemModelNodeId())->Definition()->GetNumberOfOutcomes();
//    ROS_INFO("%s AddModelLink(): num of states: %d", this->CLASSNAME, numOfSystemStates);

    if (modelLink.GetStateMapping().size() != numOfSystemStates) {
      ROS_ERROR("%s AddModelLink(): state mapping size does not match. Seems that the size of the comp node states (%d) and the system node states (%d) are different. Skip this link",
                this->CLASSNAME, modelLink.GetStateMapping().size(), numOfSystemStates);
    }

  } else {
    ROS_ERROR("%s: AddModelLink(): no state mapping is given. Skip this link", this->CLASSNAME);
    return;
  }

  //check if the element is already in list
  bool foundIdentical = false;
  for (vector<BayesianML>::iterator it=this->modelLinks.begin(); it != this->modelLinks.end(); ++it) {

    //checks only if a element with identical system model node id exists
    if (it->GetSystemModelNodeId() == modelLink.GetSystemModelNodeId()) {
      foundIdentical = true;
      break;
    }
  }

  //if none was found ... add it
  if (!foundIdentical) {
    this->modelLinks.push_back(modelLink);

    ROS_INFO("%s: AddModelLink(): set model link with system model node (id: %d, name %s) to reflect comp model node (id: %d, name: %s) for comp model (model id: %d)",
             this->CLASSNAME, modelLink.GetSystemModelNodeId(), modelLink.GetSystemModelNodeName().c_str(),
              modelLink.GetCompModelNodeId(), modelLink.GetCompModelNodeName().c_str(), modelLink.GetModelId());
  } else {
    ROS_ERROR("%s: AddModelLink(): Link element that targets identical node in the system model already exists in the list. Do not add it!", this->CLASSNAME);
  }


}

void BayesianKB::SetSystemModelNodeDefaultDef (const string nodeName, double prob1, double prob2)
{

  int systemNodeId = this->systemModel->GetDSLNetwork()->FindNode(nodeName.c_str());;

  cout << "DEBUG: SetSystemModelNodeDefaultDef: node id: " <<  systemNodeId << endl;
  DSL_doubleArray theProbs;
  theProbs.SetSize(2);

  theProbs[0] = prob1;
  theProbs[1] = prob2;

  this->systemModel->GetDSLNetwork()->GetNode(systemNodeId)->Definition()->SetDefinition(theProbs);

  /*
  //get the values again to check ... no use because values are not the same as the definition
  DSL_sysCoordinates theCoordinates( *(this->systemModel->GetDSLNetwork()->GetNode(systemNodeId)->Value()) );

  double p1 = 0.81;

  theCoordinates[0] = 0;
  theCoordinates.GoToCurrentPosition();
  p1 = theCoordinates.UncheckedValue();

  double p2 = 0.82;
  theCoordinates[0] = 1;
  theCoordinates.GoToCurrentPosition();
  p2 = theCoordinates.UncheckedValue();

  ROS_INFO("%s: SetSystemModelNodeDefaultDef(): update model id: %d, node %s, prop1: %f prop2: %f",
           this->CLASSNAME, this->currentModelId, nodeName.c_str(), p1, p2);
  */
}

void BayesianKB::UpdateSystemModel(DSLModel* dslModel, long modelId) {

  if (this->systemModel == NULL) {
    ROS_ERROR("%s: UpdateSystemModel(): system model not set. No update possible.", this->CLASSNAME);
    return;
  }

  int systemNodeId;
  string nodeName;

  //find link element that corresponds to the corrent comp model
  for (vector<BayesianML>::iterator it = this->modelLinks.begin(); it != this->modelLinks.end(); ++it) {

    // update only elemtents of the current mdoel
    if (it->GetModelId() == modelId) { //this->currentModelId

      //Update the system model with the updated probability
      systemNodeId = it->GetSystemModelNodeId();
      nodeName = it->GetSystemModelNodeName();

      //TODO: check the right meaning (order?) ... failure and operational!

      //get the probs
      int currentSliceIndex = dslModel->GetCurrentSliceIndex();
      int compModelNodeId = it->GetCompModelNodeId();
      DSL_Dmatrix* matrix =  dslModel->GetDSLNetwork()->GetNode(compModelNodeId)->Value()->GetMatrix();

      //state mapping
      DSL_doubleArray theProbs;
      unsigned short numOfStates = it->GetStateMapping().size();
      theProbs.SetSize(numOfStates);

      for (int i=0; i<numOfStates ; i++) {
        // e.g. GetStateMapping() ... comp states [1, 3, 2] -> sys states [1,2,3]
        unsigned short compState = it->GetStateMapping()[i];
        double compStateProp = (*matrix)[currentSliceIndex * 2 + compState];
        theProbs[i] = compStateProp;
        //comp node state

        ROS_DEBUG("%s: UpdateSystem(): update model id: %d, comp state: %d to sys state: %d, prop: %f ",
                 this->CLASSNAME, modelId, compState, i, compStateProp);
      }

      this->systemModel->GetDSLNetwork()->GetNode(systemNodeId)->Definition()->SetDefinition(theProbs);

      // break;

      //temp test code: show set probs

      //get the values again to check ...
      DSL_sysCoordinates theCoordinates( *(this->systemModel->GetDSLNetwork()->GetNode(systemNodeId)->Value()) );

      double prob1 = 0.81;

      theCoordinates[0] = 0;
      theCoordinates.GoToCurrentPosition();
      prob1 = theCoordinates.UncheckedValue();

      double prob2 = 0.82;
      theCoordinates[0] = 1;
      theCoordinates.GoToCurrentPosition();
      prob2 = theCoordinates.UncheckedValue();

      ROS_DEBUG("%s: UpdateSystem(): update model id: %d, node %s, prop1: %f prop2: %f",
               this->CLASSNAME, this->currentModelId, nodeName.c_str(), prob1, prob2);

    }
  }


    //temp test code:

//    //update net
//    this->systemModel->GetDSLNetwork()->UpdateBeliefs();
//
//    //get the overall prob
//    int robotId = this->systemModel->GetDSLNetwork()->FindNode("Robot");
//    DSL_sysCoordinates robotVal ( *(this->systemModel->GetDSLNetwork()->GetNode(robotId)->Value()) );
//    robotVal[0] = 0;
//    robotVal.GoToCurrentPosition();
//    double prob11 = robotVal.UncheckedValue();
//
//    robotVal[0] = 1;
//    robotVal.GoToCurrentPosition();
//    double prob22 = robotVal.UncheckedValue();
//
//    ROS_INFO("%s: UpdateSystem(): update model id: %d, node Robot, prop1: %f prop2: %f",
//             this->CLASSNAME, this->currentModelId, prob11, prob22);


}

void BayesianKB::InferSystemModel() {

  if (this->systemModel == NULL) {
    ROS_ERROR("%s: InferSystemModel(): system model not set. No inference possible.", this->CLASSNAME);
    return;
  }

  //update net
  this->systemModel->GetDSLNetwork()->UpdateBeliefs();

  //get the overall prob
  int robotId = this->systemModel->GetDSLNetwork()->FindNode("Robot");
  DSL_sysCoordinates robotVal ( *(this->systemModel->GetDSLNetwork()->GetNode(robotId)->Value()) );
  robotVal[0] = 0;
  robotVal.GoToCurrentPosition();
  double prob11 = robotVal.UncheckedValue();

  robotVal[0] = 1;
  robotVal.GoToCurrentPosition();
  double prob22 = robotVal.UncheckedValue();

  ROS_INFO("%s: InferSystemModel(): update model id: %d, node Robot, prop1: %f prop2: %f",
           this->CLASSNAME, this->currentModelId, prob11, prob22);

}
