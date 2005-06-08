
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet Lagadic / IRISA-INRIA Rennes, 2005
 * www  : http://www.irisa.fr/lagadic
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      vpMatrixException.h
 * Project:   ViSP2
 * Author:    Eric Marchand
 *
 * Version control
 * ===============
 *
 *  $Id: vpMatrixException.h,v 1.1.1.1 2005-06-08 07:08:06 fspindle Exp $
 *
 * Description
 * ============
 *  error that can be emited by the vpMatrix class and its derivates
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/


#ifndef __vpMatrixException_H
#define __vpMatrixException_H


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

/* \brief error that can be emited by the vpMatrix class and its derivates
 */
class vpMatrixException : public vpException
{
public:
  /*!
    \brief Lists the possible error than can be emmited while calling
    vpMatrix member
   */
  enum errorCodeEnum
    {
      //! error returns by a constructor
      constructionorERR,
      //! something is not initialized
      notInitializedERR,
      //! function not implemented
      notImplementedERR,
      //! index out of range
      outOfRangeERR,
      //! iterative algorithm doesn't converge (ex SVD)
      convergencyERR,
      incorrectMatrixSizeERR,
      forbiddenOperatorERR,
      subMatrixERR,
      matrixERR
    } ;

public:
  vpMatrixException (const int code, const char * msg)
    : vpException(code, msg){ ; }
  vpMatrixException (const int code, const string & msg)
    : vpException(code, msg){ ; }
  vpMatrixException (const int code)
    : vpException(code){ ; }
 // vpMatrixException() : vpException() { ;}
};





#endif /* #ifndef __vpMatrixException_ERROR_H */


/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
