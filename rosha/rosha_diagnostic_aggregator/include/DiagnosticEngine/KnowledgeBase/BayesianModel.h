/*
 * BayesianModel.h
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
 *  Created on: Feb 5, 2013		9:57:19 AM
 *      Author: Dominik Kirchner
 */

#ifndef BAYESIANMODEL_H_
#define BAYESIANMODEL_H_

#include <DiagnosticEngine/KnowledgeBase/Model.h>

namespace diagnostic_engine {

class BayesianModel : public Model {

  /*
   * specific things for an bayesian way of knowledge representation
   */

public:
  BayesianModel();
  ~BayesianModel();

private:


};
}



#endif /* BAYESIANMODEL_H_ */
