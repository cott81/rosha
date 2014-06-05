/*
 * BaseRepair.h
 *
 * Copyright 2012 Carpe Noctem, Distributed Systems Group,
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
 *  Created on: Sep 6, 2012		12:31:53 PM
 *      Author: Dominik Kirchner
 */

#ifndef BASEREPAIR_H_
#define BASEREPAIR_H_

#include <iostream>

namespace gen_repair_plugins
{
  class BaseRepair
  {
    public:
      virtual void initialize() = 0;
      virtual ~BaseRepair(){}
      virtual void Repair() {
        std::cout << "BaseRepair: empty Repair" << std::endl;
      }
      virtual std::string GetName() {
        return "Error: Base Clase func call";
      }

    protected:
      BaseRepair(){}

  };
};


#endif /* BASEREPAIR_H_ */
