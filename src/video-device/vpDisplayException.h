
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet Lagadic / IRISA-INRIA Rennes, 2005
 * www  : http://www.irisa.fr/lagadic
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      vpDisplayException.h
 * Project:   ViSP2
 * Author:    Eric Marchand
 *
 * Version control
 * ===============
 *
 *  $Id: vpDisplayException.h,v 1.4 2005-12-05 10:19:09 marchand Exp $
 *
 * Description
 * ============
 *  error that can be emited by the vpDisplay class and its derivates
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/


#ifndef __vpDisplayException_H
#define __vpDisplayException_H


/* ------------------------------------------------------------------------- */
/* --- INCLUDE ------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */


/* \file vpDisplayException.h
   \brief error that can be emited by the vpDisplay class and its derivates
 */
/* Classes standards. */
#include <iostream>                /* Classe ostream.    */
#include <string>                  /* Classe string.     */
#include <visp/vpException.h>

using namespace std;

/* ------------------------------------------------------------------------- */
/* --- CLASS --------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */

/* \brief error that can be emited by the vpDisplay class and its derivates
 */
class vpDisplayException : public vpException
{
public:
  /*!
    \brief Lists the possible error than can be emmited while calling
    vpDisplay member
   */
  enum errorDisplayCodeEnum
    {
      notInitializedError,
      cannotOpenWindowError,
      connexionError,
      XWindowsError,
      GTKWindowsError,
      colorAllocError,
      depthNotSupportedError
    } ;

public:
  vpDisplayException (const int code, const char * msg)
    : vpException(code, msg){ ; }
  vpDisplayException (const int code, const string & msg)
    : vpException(code, msg){ ; }
  vpDisplayException (const int code)
    : vpException(code){ ; }

};





#endif /* #ifndef __vpDisplayException_H */


/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
