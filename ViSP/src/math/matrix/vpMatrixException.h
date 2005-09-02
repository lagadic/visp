
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
 *  $Id: vpMatrixException.h,v 1.3 2005-09-02 14:35:17 fspindle Exp $
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
//using namespace std;
#include <iostream>                /* Classe ostream.    */
#include <string>                  /* Classe string.     */
#include <visp/vpException.h>
using namespace std;

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
      constructionError,
      //! something is not initialized
      notInitializedError,
      //! function not implemented
      notImplementedError,
      //! index out of range
      outOfRangeError,
      //! iterative algorithm doesn't converge (ex SVD)
      convergencyError,
      incorrectMatrixSizeError,
      forbiddenOperatorError,
      subMatrixError,
      matrixError
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
