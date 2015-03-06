/*
 * CompFailureConf.h
 *
 *  Created on: Mar 6, 2015
 *      Author: dominik
 */

#ifndef COMPFAILURECONF_H_
#define COMPFAILURECONF_H_

#include "ros/ros.h"
#include "Defs.h"

namespace error_seeder
{

class CompFailureConf
{
public:
  CompFailureConf();
  virtual ~CompFailureConf();

  int getCompId() const
  {
    return compId;
  }

  void setCompId(int compId)
  {
    this->compId = compId;
  }


  double GetFailureProb(ErrorId failureId );
  void SetFailureProb(ErrorId failureId, double prob );



private:
  int compId;
  double probNP;
  double probAR;
  double probDL;
  double probEL;
  double probEX;

  /*!
   * \brief Check if the given probability is valid (has the correct value domain)
   *
   * \return true if it is valid, false otherwise
   */
  bool IsProbValid(double prob);


};

} /* namespace error_seeder */

#endif /* COMPFAILURECONF_H_ */
