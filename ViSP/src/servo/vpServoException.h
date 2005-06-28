
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet Lagadic / IRISA-INRIA Rennes, 2005
 * www  : http://www.irisa.fr/lagadic
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      vpServoException.h
 * Project:   ViSP2
 * Author:    Eric Marchand
 *
 * Version control
 * ===============
 *
 *  $Id: vpServoException.h,v 1.2 2005-06-28 12:52:55 marchand Exp $
 *
 * Description
 * ============
 *  error that can be emited by the vpServo class and its derivates
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/


#ifndef __vpServoException_H
#define __vpServoException_H

/*!
  \file vpServoException.h
  \brief error that can be emited by the vpServo class and its derivates
*/


using namespace std;

#include <iostream>                /* Classe ostream.    */
#include <string>                  /* Classe string.     */
#include <visp/vpException.h>

/* ------------------------------------------------------------------------- */
/* --- CLASS --------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */

/* \class vpServoException
   \brief error that can be emited by the vpServo class and its derivates
   \author Eric Marchand   (Eric.Marchand@irisa.fr) Irisa / Inria Rennes
 */
class vpServoException : public vpException
{
public:
  /*!

    \brief Lists the possible error than can be emmited while calling
    vpServo member
   */
  enum errorServoCodeEnum
    {
      //! feature list or desired feature list is empty
      noFeatureError,
      //! other exception
      servoError
    } ;

public:
  vpServoException (const int code, const char * msg)
    : vpException(code, msg){ ; }
  vpServoException (const int code, const string & msg)
    : vpException(code, msg){ ; }
  vpServoException (const int code)
    : vpException(code){ ; }

};





#endif /* #ifndef __vpServoException_ERROR_H */


/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
