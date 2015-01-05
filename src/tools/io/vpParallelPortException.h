/****************************************************************************
 *
 * $Id$
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2014 by INRIA. All rights reserved.
 * 
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ViSP with software that can not be combined with the GNU
 * GPL, please contact INRIA about acquiring a ViSP Professional 
 * Edition License.
 *
 * See http://www.irisa.fr/lagadic/visp/visp.html for more information.
 * 
 * This software was developed at:
 * INRIA Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 * http://www.irisa.fr/lagadic
 *
 * If you have questions regarding the use of this file, please contact
 * INRIA at visp@inria.fr
 * 
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
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

#include <visp/vpException.h>

#include <iostream>                /* Classe std::ostream.    */
#include <string>                  /* Classe string.     */

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
    vpParallelPortException (const int id,  const char* format, ...)
    {
      this->code = id;
      va_list args;
      va_start(args, format);
      setMessage(format, args);
      va_end (args);
    }
    vpParallelPortException (const int id, const std::string & msg)
      : vpException(id, msg){ ; }
    vpParallelPortException (const int id)
      : vpException(id){ ; }

};

#endif
