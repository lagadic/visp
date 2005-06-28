
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet Lagadic / IRISA-INRIA Rennes, 2005
 * www  : http://www.irisa.fr/lagadic
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      vpImageException.h
 * Project:   ViSP2
 * Author:    Eric Marchand
 *
 * Version control
 * ===============
 *
 *  $Id: vpImageException.h,v 1.2 2005-06-28 13:30:44 marchand Exp $
 *
 * Description
 * ============
 *  error that can be emited by the vpImage class and its derivates
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/


#ifndef __vpImageException_H
#define __vpImageException_H


/* ------------------------------------------------------------------------- */
/* --- INCLUDE ------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */


/* \file vpImageException.h
   \brief error that can be emited by the vpImage class and its derivates
 */
/* Classes standards. */
using namespace std;
#include <iostream>                /* Classe ostream.    */
#include <string>                  /* Classe string.     */
#include <visp/vpException.h>

/* ------------------------------------------------------------------------- */
/* --- CLASS --------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */

/* \brief error that can be emited by the vpImage class and its derivates
 */
class vpImageException : public vpException
{
public:
  /*!
    \brief Lists the possible error than can be emmited while calling
    vpImage member
   */
  enum errorImageCodeEnum
    {
      ioError,
      noFileNameError,
      notInitializedError,
      incorrectInitializationError
    } ;

public:
  vpImageException (const int code, const char * msg)
    : vpException(code, msg){ ; }
  vpImageException (const int code, const string & msg)
    : vpException(code, msg){ ; }
  vpImageException (const int code)
    : vpException(code){ ; }

};





#endif /* #ifndef __vpImageException_H */


/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
