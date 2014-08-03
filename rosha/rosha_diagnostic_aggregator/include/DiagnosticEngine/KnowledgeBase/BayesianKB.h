/*
 * BayesianKB.h
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
 *  Created on: Jan 31, 2013		3:28:54 PM
 *      Author: Dominik Kirchner
 */

#ifndef BAYESIANKB_H_
#define BAYESIANKB_H_

//#pragma message ("BayesianKB")


#include <DiagnosticEngine/KnowledgeBase/DSLModel.h>
#include <DiagnosticEngine/KnowledgeBase/BayesianKC.h>
#include <AnalyzerHelper/ObservationQueue.h>

#include <DiagnosticEngine/KnowledgeBase/KnowledgeBase.h>
#include <DiagnosticEngine/KnowledgeBase/Model.h>
#include <DiagnosticEngine/Reasoner/Reasoner.h>
#include <DiagnosticEngine/KnowledgeBase/ModelFactory.h>

#include <DiagnosticEngine/DiagnosticEngine.h>
#include <DiagnosticEngine/KnowledgeBase/BayesianML.h>

#include <boost/thread/mutex.hpp>

#include <ros/ros.h>

namespace diagnostic_engine {

//class KnowledgeBase;

class BayesianKB : public KnowledgeBase {

  //inheritance is by default private ... so is the constructor

  //TODO: new inhertiance ... HirachicalBayesianKB ... use this for a splitted model
  // datastructure: graph (-> graph of graphs (PartModels)) ... or merge all to one graph, problem with mulitple instances of partModels?

public:
  BayesianKB();
  ~BayesianKB();
  void Init(ModelFactory* modelFactory);

  //interface
  long AddModel(const std::string& fromFileName, bool useAsSystemModel=false ,bool useModelsDefaultHistorySettings=true);
  //uses the hash of the model name as the key of the map in which the model is stored
  long AddModel(const std::string& modelName, const std::string& fromFileName, bool useModelsDefaultHistorySettings=true);
  virtual void SetHistorySettings(unsigned int numOfHistorySlices,
                                  unsigned int sliceTime,
                                  unsigned int currentTimeSlice,
                                  long modelId=-1);
  void Update(std::vector<KnowledgeChange*> kcs); //call be ref?
  void UpdateTime(unsigned long currentTime);    //update the current time in the knowledge base
  void UpdateTime(unsigned long currentTime, int modelId); //update the current time for on given model
  void Accept(Reasoner& visitor); //accept a type of visitor
  //void RestructureModel(); //TODO: later

  /*!
   * \brief Returns the specified model.
   *
   * Returns the given model as an abstract model.
   *
   * \param modelId specifies the model by an id (only for component models)
   * \param useSystemModel=false returns the system model. modelId is than ignored.
   *
   * \return the abstract model
   */
  Model* GetModel(long modelId, bool useSystemModel=false);


  //from splitableKB
  /*!
   * \brief Sets a specific model to the current model in the knowledge base.
   *
   * Sets the given model to the current model in the knowledge base. This model is used for
   * inference.
   *
   * \param modelId specifies the (component) model to be used
   * \param useSystemModel=false uses the system model. The parameter modelId is than ignored
   */
  void SetCurrentModel(long modelId, bool useSystemModel=false);

  //from mergeableKB (inher from splitableKB)
  //void MergeModels()
//  void SetSystemModel(const std::string& fromFileName, bool useModelsDefaultHistorySettings=true);

  /*!
   * \brief Adds a model link between a component model and the system model to the map
   *
   * Adds a link between a target node (node of interest) of a component model and a corresponding node of the system model.
   * The function checks the given infos (like name or id) and complements missing ones, e.g. if only a system node name is given,
   * the system node id is checked and added. In case no info of the system node is provided (no id nor name), we assume that the
   * name of the given component node is identical with the system node.
   *
   * \param modelLink the link object that specifies the exact model node ids and the component model id.
   */
  void AddModelLink(BayesianML modelLink);

  /*!
   * \brief Sets the probabilities of a node in the system model to given values
   *
   * Sets the definition of a node of the system model hard to given values. This function is used with nodes that have t
   * two states only. Porpose: reset nodes, that are not used any more to its default values.
   */
  void SetSystemModelNodeDefaultDef (const string nodeName, double prob1, double prob2);


private:

  const char* CLASSNAME;
  //add generic multi model support??? ... interface splitable(sub/compModels), mergable (able to merge
  //to the one complete model

  //from splitable
  std::map<long, DSLModel*> compModels; //or better a map to access directly per id?
  long currentModelId;
  int bayesianCMCount;

  DSLModel* systemModel;
  std::vector<BayesianML> modelLinks;        /*!< specifies the links between nodes of the component models and nodes of the system model (key of component model, model link object)   */

  //history time settings
  unsigned int numOfHistorySlices;
  unsigned int sliceTime;
  unsigned int currentTimeSlice;

  void UpdateSystemModel(DSLModel* dslModel, long modelId);
  // temp ...
  void InferSystemModel();

  boost::mutex mutexObj;

};
}

#endif /* BAYESIANKB_H_ */
