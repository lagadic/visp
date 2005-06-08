
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet Lagadic / IRISA-INRIA Rennes, 2005
 * www  : http://www.irisa.fr/lagadic
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      vpSimulatorException.h
 * Project:   ViSP2
 * Author:    Eric Marchand
 *
 * Version control
 * ===============
 *
 *  $Id: vpSimulatorException.h,v 1.1.1.1 2005-06-08 07:08:14 fspindle Exp $
 *
 * Description
 * ============
 *  error that can be emited by the vpSimulator class and its derivates
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/


#ifndef __vpSimulatorException_H
#define __vpSimulatorException_H


/* ------------------------------------------------------------------------- */
/* --- INCLUDE ------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */


/* \file vpSimulatorException.h
   \brief error that can be emited by the vpSimulator class and its derivates
 */
/* Classes standards. */
using namespace std;
#include <iostream>                /* Classe ostream.    */
#include <string>                  /* Classe string.     */
#include <visp/vpException.h>

/* ------------------------------------------------------------------------- */
/* --- CLASS --------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */

/* \brief error that can be emited by the vpSimulator class and its derivates
 */
class vpSimulatorException : public vpException
{
public:
  /*!
    \brief Lists the possible error than can be emmited while calling
    vpSimulator member
   */
  enum errorSimulatorCodeEnum
    {
      ioERR,
      noFileNameERR,
      notInitializedERR,
      windowSizeNotInitializedERR,
      badInitializationERR
    } ;

public:
  vpSimulatorException (const int code, const char * msg)
    : vpException(code, msg){ ; }
  vpSimulatorException (const int code, const string & msg)
    : vpException(code, msg){ ; }
  vpSimulatorException (const int code)
    : vpException(code){ ; }

};





#endif /* #ifndef __vpSimulatorException_H */


/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
