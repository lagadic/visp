/****************************************************************************
 *
 * $Id$
 *
 * Copyright (C) 1998-2010 Inria. All rights reserved.
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


/*!
  \file vpIoException.h
  \brief Error that can be emited by the vpIoTools class and its derivates.
*/

#include <iostream>
#include <string>

#include <visp/vpConfig.h>
#include <visp/vpException.h>



/* ------------------------------------------------------------------------- */
/* --- CLASS --------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */

/*!
  \class vpIoException
  \ingroup Exception
  \brief Error that can be emited by the vpIoTools class and its derivates.
 */
class VISP_EXPORT vpIoException : public vpException
{
public:
  /*!
    \brief Lists the possible error than can be emmited while calling
    vpIo member.
   */
  enum error
    {
      invalidDirectoryName, /*! Directory name is invalid. */
      cantCreateDirectory,  /*! Unable to create a directory. */
      cantGetUserName,      /*! User name is not available. */
      cantGetenv            /*! Cannot get environment variable value. */
    } ;

public:
  vpIoException (const int code, const char * msg)
    : vpException(code, msg){ ; }
  vpIoException (const int code, const std::string & msg)
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
