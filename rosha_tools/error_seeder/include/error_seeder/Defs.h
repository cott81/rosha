/*
 * Defs.h
 *
 *  Created on: May 28, 2014
 *      Author: dominik
 */

#ifndef DEFS_H_
#define DEFS_H_


  enum ErrorId
  {
    UNDEFINED=-1,
    NULLPOINTER=1,
    ARRAYINDEXERROR,
    ENDLESSLOOP,
    STOP_ENDLESSLOOP,
    DEADLOCK,
    LEASEDEADLOCK,
    ENABLE_PROBABILITY,
    GENERAL_EXCEPTION,
    DISABLE_PROBABILITY,
    SHUTDOWNPROCESS,
    RECOMPUTEMTBF
  };




#endif /* DEFS_H_ */
