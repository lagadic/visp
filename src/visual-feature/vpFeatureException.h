
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet Lagadic / IRISA-INRIA Rennes, 2005
 * www  : http://www.irisa.fr/lagadic
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      vpFeatureException.h
 * Project:   ViSP2
 * Author:    Eric Marchand
 *
 * Version control
 * ===============
 *
 *  $Id: vpFeatureException.h,v 1.3 2005-09-02 14:35:18 fspindle Exp $
 *
 * Description
 * ============
 *  error that can be emited by the vpFeature class and its derivates
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/


#ifndef __vpFeatureException_H
#define __vpFeatureException_H


/* ------------------------------------------------------------------------- */
/* --- INCLUDE ------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */


/* \file vpFeatureException.h
   \brief error that can be emited by the vpFeature class and its derivates
 */
/* Classes standards. */
#include <iostream>                /* Classe ostream.    */
#include <string>                  /* Classe string.     */
#include <visp/vpException.h>

using namespace std;

/* ------------------------------------------------------------------------- */
/* --- CLASS --------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */

/* \brief error that can be emited by the vpFeature class and its derivates
 */
class vpFeatureException : public vpException
{
public:
  /*!
    \brief Lists the possible error than can be emmited while calling
    vpFeature member
   */
  enum errorFeatureCodeEnum
    {
      //! feature list or desired feature list is empty
      badErrorVectorError,
      sizeMismatchError,
      notInitializedError,
      badInitializationError
    } ;

public:
  vpFeatureException (const int code, const char * msg)
    : vpException(code, msg){ ; }
  vpFeatureException (const int code, const string & msg)
    : vpException(code, msg){ ; }
  vpFeatureException (const int code)
    : vpException(code){ ; }

};





#endif /* #ifndef __vpFeatureException_H */


/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
