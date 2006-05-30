/****************************************************************************
 *
 * $Id: vpIoException.h,v 1.3 2006-05-30 08:40:46 fspindle Exp $
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
 * This file is part of the ViSP toolkit.
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
 * Exceptions that can be emited by the vpIo class and its derivates.
 *
 * Authors:
 * Eric Marchand
 *
 *****************************************************************************/


#ifndef __vpIoException_H
#define __vpIoException_H


/* ------------------------------------------------------------------------- */
/* --- INCLUDE ------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */


/* \file vpIoException.h
   \brief error that can be emited by the vpIo class and its derivates
 */
/* Classes standards. */
#include <iostream>                /* Classe ostream.    */
#include <string>                  /* Classe string.     */
#include <visp/vpConfig.h>
#include <visp/vpException.h>

using namespace std;

/* ------------------------------------------------------------------------- */
/* --- CLASS --------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */

/* \brief error that can be emited by the vpIo class and its derivates
 */
class VISP_EXPORT vpIoException : public vpException
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
