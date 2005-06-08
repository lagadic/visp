
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet Lagadic / IRISA-INRIA Rennes, 2005
 * www  : http://www.irisa.fr/lagadic
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      vpIoException.h
 * Project:   ViSP2
 * Author:    Eric Marchand
 *
 * Version control
 * ===============
 *
 *  $Id: vpIoException.h,v 1.1.1.1 2005-06-08 07:08:11 fspindle Exp $
 *
 * Description
 * ============
 *  error that can be emited by the vpIo class and its derivates
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/


#ifndef __vpIoException_H
#define __vpIoException_H


/* ------------------------------------------------------------------------- */
/* --- INCLUDE ------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */


/* \file vpIoException.h
   \brief error that can be emited by the vpIo class and its derivates
 */
/* Classes standards. */
using namespace std;
#include <iostream>                /* Classe ostream.    */
#include <string>                  /* Classe string.     */
#include <visp/vpException.h>

/* ------------------------------------------------------------------------- */
/* --- CLASS --------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */

/* \brief error that can be emited by the vpIo class and its derivates
 */
class vpIoException : public vpException
{
public:
  /*!
    \brief Lists the possible error than can be emmited while calling
    vpIo member
   */
  enum errorIoCodeEnum
    {
      //! feature list or desired feature list is empty
      ERRInvalidDirectoryName,
      ERRCantStatDirectory,
      ERRNotADirectory,
      ERRNotWritable,
      ERRCantCreateDirectory
    } ;

public:
  vpIoException (const int code, const char * msg)
    : vpException(code, msg){ ; }
  vpIoException (const int code, const string & msg)
    : vpException(code, msg){ ; }
  vpIoException (const int code)
    : vpException(code){ ; }

};





#endif /* #ifndef __vpIoException_H */


/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
