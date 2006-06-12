/****************************************************************************
 *
 * $Id: vpException.h,v 1.5 2006-06-12 14:49:52 brenier Exp $
 *
 * Copyright (C) 1998-2006 Inria. All rights reserved.
 *
 * This software was developed at:
 * IRISA/INRIA Rennes
 * Projet Lagadic
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * http://www.irisa.fr/lagadic
 *
 * This file is part of the ViSP toolkit
 *
 * This file may be distributed under the terms of the Q Public License
 * as defined by Trolltech AS of Norway and appearing in the file
 * LICENSE included in the packaging of this file.
 *
 * Licensees holding valid ViSP Professional Edition licenses may
 * use this file in accordance with the ViSP Commercial License
 * Agreement provided with the Software.
 *
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Contact visp@irisa.fr if any conditions of this licensing are
 * not clear to you.
 *
 * Description:
 * Exception handling.
 *
 * Authors:
 * Nicolas Mansard
 *
 *****************************************************************************/


/* \file vpException.h
   \brief error that can be emited by the vp class and its derivates
 */

#ifndef __vpException_H
#define __vpException_H


/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */


/* Classes standards. */
#include <iostream>                /* Classe ostream.    */
#include <string>                  /* Classe string.     */

#include <visp/vpConfig.h>

using namespace std;

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */


/* \class vpException
   \brief error that can be emited by the vp class and its derivates
 */
class VISP_EXPORT vpException //: public std::exception
{

private:

  //! Contains the error code, see the errorCodeEnum table for details.
  int code;

  //! Contains an error message (can be empty)
  string message;

private:

  //!  forbid the empty constructor (private)
  vpException();

public:

  enum generalExceptionEnum
    {
      memoryAllocationError,
      memoryFreeError,
      functionNotImplementedError,
      ioError,
      cannotUseConstructorError,
      notImplementedError,
      divideByZeroError
    } ;
  // ~vpException() throw() {;}

  vpException (const int code, const char * msg);
  vpException (const int code, const string & msg);
  vpException (const int code);

  //!  send the object code
  int getCode (void);

  //! send a reference (constant) related the error message (can be empty)
  const string &getStringMessage (void);
  //! send a pointer on the array of  \e char related to the error string.
  //!Cannot be  \e NULL.
  const char *getMessage (void);

  //! print the error structure
  friend VISP_EXPORT ostream & operator << (ostream & os,
				const vpException & art);

};





#endif /* #ifndef __vpException_H */


/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
