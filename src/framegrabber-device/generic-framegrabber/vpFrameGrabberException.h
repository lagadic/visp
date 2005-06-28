
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet Lagadic / IRISA-INRIA Rennes, 2005
 * www  : http://www.irisa.fr/lagadic
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      vpFrameGrabberException.h
 * Project:   ViSP2
 * Author:    Eric Marchand
 *
 * Version control
 * ===============
 *
 *  $Id: vpFrameGrabberException.h,v 1.2 2005-06-28 10:22:49 fspindle Exp $
 *
 * Description
 * ============
 *  error that can be emited by the vpFrameGrabber class and its derivates
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/


#ifndef __vpFrameGrabberException_H
#define __vpFrameGrabberException_H


/* ------------------------------------------------------------------------- */
/* --- INCLUDE ------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */


/* \file vpFrameGrabberException.h
   \brief error that can be emited by the vpFrameGrabber class and its derivates
 */
/* Classes standards. */
using namespace std;
#include <iostream>                /* Classe ostream.    */
#include <string>                  /* Classe string.     */
#include <visp/vpException.h>

/* ------------------------------------------------------------------------- */
/* --- CLASS --------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */

/* \brief error that can be emited by the vpFrameGrabber class and its derivates
 */
class vpFrameGrabberException : public vpException
{
public:
  /*!
    \brief Lists the possible error than can be emmited while calling
    vpFrameGrabber member
   */
  enum errorFrameGrabberCodeEnum
    {
      settingError,
      initializationError,
      otherError
    } ;

public:
  vpFrameGrabberException (const int code, const char * msg)
    : vpException(code, msg){ ; }
  vpFrameGrabberException (const int code, const string & msg)
    : vpException(code, msg){ ; }
  vpFrameGrabberException (const int code)
    : vpException(code){ ; }

};





#endif /* #ifndef __vpFrameGrabberException_H */


/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
