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
 * Exceptions that can be emited by the vpParallelPort class and its derivates.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/


#ifndef __vpParallelPortException_H
#define __vpParallelPortException_H


/* ------------------------------------------------------------------------- */
/* --- INCLUDE ------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */


/*!

  \file vpParallelPortException.h

  \brief Error that can be emited by the vpParallelPort class and its
  derivates.

*/

/* Classes standards. */
#include <iostream>                /* Classe std::ostream.    */
#include <string>                  /* Classe string.     */
#include <visp/vpConfig.h>
#include <visp/vpException.h>



/* ------------------------------------------------------------------------- */
/* --- CLASS --------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */

/*!
  \class vpParallelPortException
  \ingroup Exception

  \brief Error that can be emited by the vpParallelPort class and its
  derivates.
 */
class VISP_EXPORT vpParallelPortException : public vpException
{
public:
  /*!
    \brief Lists the possible errors than can be emmited while calling
    vpParallelPort member
   */
  enum error
    {
      opening, /*!< Cannot access to the parallel port device. */
      closing  /*!< Cannot close the parallel port device. */
    } ;

public:
  vpParallelPortException (const int code, const char * msg)
    : vpException(code, msg){ ; }
  vpParallelPortException (const int code, const std::string & msg)
    : vpException(code, msg){ ; }
  vpParallelPortException (const int code)
    : vpException(code){ ; }

};





#endif /* #ifndef __vpParallelPortException_H */


/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
