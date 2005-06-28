
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet Lagadic / IRISA-INRIA Rennes, 2005
 * www  : http://www.irisa.fr/lagadic
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      vpTrackingException.h
 * Project:   ViSP2
 * Author:    Eric Marchand
 *
 * Version control
 * ===============
 *
 *  $Id: vpTrackingException.h,v 1.2 2005-06-28 13:08:00 marchand Exp $
 *
 * Description
 * ============
 *  error that can be emited by the vpTracking class and its derivates
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/


#ifndef __vpTrackingException_H
#define __vpTrackingException_H


/* ------------------------------------------------------------------------- */
/* --- INCLUDE ------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */


/* \file vpTrackingException.h
   \brief error that can be emited by the vpTracking class and its derivates
 */
/* Classes standards. */
using namespace std;
#include <iostream>                /* Classe ostream.    */
#include <string>                  /* Classe string.     */
#include <visp/vpException.h>

/* ------------------------------------------------------------------------- */
/* --- CLASS --------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */

/* \brief error that can be emited by the vpTracking class and its derivates
 */
class vpTrackingException : public vpException
{
public:
  /*!
    \brief Lists the possible error than can be emmited while calling
    vpTracking member
   */
  enum errorTrackingCodeEnum
    {
      featureLostError,

      // Moving edges
      notEnoughPointError,
      initializationError,
      fatalError
    } ;

public:
  vpTrackingException (const int code, const char * msg)
    : vpException(code, msg){ ; }
  vpTrackingException (const int code, const string & msg)
    : vpException(code, msg){ ; }
  vpTrackingException (const int code)
    : vpException(code){ ; }

};





#endif /* #ifndef __vpTrackingException_H */


/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
