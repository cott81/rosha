/*
 * KnowledgeBase.h
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
 *  Created on: Jan 31, 2013		1:45:12 PM
 *      Author: Dominik Kirchner
 */

#ifndef KNOWLEDGEBASE_H_
#define KNOWLEDGEBASE_H_
//#pragma message ("KnowledgeBase")

#include "DiagnosticEngine/KnowledgeBase/KnowledgeChange.h"
#include <DiagnosticEngine/KnowledgeBase/Model.h>
//TODO: changes this to an abstract ModelLink object
#include <DiagnosticEngine/KnowledgeBase/BayesianML.h>
#include <DiagnosticEngine/Reasoner/Reasoner.h>
#include <DiagnosticEngine/KnowledgeBase/ModelFactory.h>
#include "ros/ros.h" //inheritance of includes? ... open question

namespace diagnostic_engine {

class KnowledgeBase {

/*
 * Knowledge Base: abstract factory pattern
 * consists of a knowledge representation (Model) (the product)
 * and an interface (methods) to manage the knowledge
 */

public:
  virtual ~KnowledgeBase(){} //to call really all desctructors
  virtual void Init(ModelFactory* modelFactory) = 0;

  //interface

  /*! \brief Add a model to the knowledge base.
   *
   * Adds a model to the knowledge base. This function can be used to add component models and system model.
   *
   * \param fromFileName the file where the model is stored
   * \param useAsSystemModel flag to mark if the model to be added is a component model or a system model (default false)
   * \param useModelsDefaultHistorySettings flag to mark the use of a default history setting for a Dynamic Bayesian Network (default true)
   *
   * \return provides the model id
   *
   */
  virtual long AddModel(const std::string& fromFileName, bool useAsSystemModel=false, bool useModelsDefaultHistorySettings=true);

  //uses the hash of the model name as the key of the map in which the model is stored
  virtual long AddModel(const std::string& modelName, const std::string& fromFileName, bool useModelsDefaultHistorySettings=true);
  //GLOBAL change for all models to create later! Early created models keep their settings. Be carefully.
  virtual void SetHistorySettings(unsigned int numOfHistorySlices,
                                  unsigned int sliceTime,
                                  unsigned int currentTimeSlice,
                                  long modelId=-1);
  virtual void Update(std::vector<KnowledgeChange*> kcs) = 0;
  virtual void UpdateTime(unsigned long currentTime);

  //TODO:drawback ... can't change easily the reasoner for a specific KnowledgeBase -> visitor
  //virtual void Infer(std::vector<ReasonerQuery> query) = 0;
  //TODO: skip visitor pattern ... -> problem cycle include
  virtual void Accept(Reasoner& visitor){}; //accept a visitor
  virtual void Infer(Reasoner& visitor){}; //accept a visitor

  //virtual void RestructureModel(); //TODO: later ...
  virtual Model* GetModel(long modelId, bool useSystemModel=false) = 0; //template? or abstract class?
  virtual void SetCurrentModel(long modelId, bool useSystemModel=false){}

  virtual void AddModelLink(BayesianML modelLink);

  virtual ModelFactory* GetModelFactory() {return this->modelFactory;}

  unsigned long hash(const char *str);


protected:
  //data model for knowledge representation
  ModelFactory* modelFactory;
  Model* model;         //abstract class: current comp model (if multiple)

private:
  const char* CLASSNAME; // = "KnowledgeBase";



};
}

#endif /* KNOWLEDGEBASE_H_ */
