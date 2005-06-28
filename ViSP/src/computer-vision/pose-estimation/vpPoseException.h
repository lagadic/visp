
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet Lagadic / IRISA-INRIA Rennes, 2005
 * www  : http://www.irisa.fr/lagadic
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      vpPoseException.h
 * Project:   ViSP2
 * Author:    Eric Marchand
 *
 * Version control
 * ===============
 *
 *  $Id: vpPoseException.h,v 1.2 2005-06-28 13:18:53 marchand Exp $
 *
 * Description
 * ============
 *  error that can be emited by the vpPose class and its derivates
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/


#ifndef __vpPoseException_H
#define __vpPoseException_H


/* ------------------------------------------------------------------------- */
/* --- INCLUDE ------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */


/* Classes standards. */
using namespace std;
#include <iostream>                /* Classe ostream.    */
#include <string>                  /* Classe string.     */
#include <visp/vpException.h>

/* ------------------------------------------------------------------------- */
/* --- CLASS --------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */

/* \brief error that can be emited by the vpPose class and its derivates
 */
class vpPoseException : public vpException
{
public:
  /*!
    \brief Lists the possible error than can be emmited while calling
    vpPose member
   */
  enum errorCodeEnum
    {
      poseError,
      //! something is not initialized
      notInitializedError,
      //! function not implemented
      notImplementedERR,
      //! index out of range
      outOfRangeError,
      notEnoughPointError
    } ;

public:
  vpPoseException (const int code, const char * msg)
    : vpException(code, msg){ ; }
  vpPoseException (const int code, const string & msg)
    : vpException(code, msg){ ; }
  vpPoseException (const int code)
    : vpException(code){ ; }
 // vpPoseException() : vpException() { ;}
};





#endif /* #ifndef __vpPoseException_ERROR_H */


/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
