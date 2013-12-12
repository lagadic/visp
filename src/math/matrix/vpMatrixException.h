/****************************************************************************
 *
 * $Id$
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2013 by INRIA. All rights reserved.
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
 * Exceptions that can be emited by the vpMatrix class and its derivates.
 *
 * Authors:
 * Eric Marchand
 *
 *****************************************************************************/

#ifndef __vpMatrixException_H
#define __vpMatrixException_H


/* ------------------------------------------------------------------------- */
/* --- INCLUDE ------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */


/* Classes standards. */
//

#include <visp/vpConfig.h>
#include <visp/vpException.h>

#include <iostream>                /* Classe std::ostream.    */
#include <string>                  /* Classe string.     */

/* ------------------------------------------------------------------------- */
/* --- CLASS --------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */

/*!
  \class vpMatrixException
  \ingroup Exception
  \brief error that can be emited by the vpMatrix class and its derivates
 */
class VISP_EXPORT vpMatrixException : public vpException
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
      matrixError,
      rankDeficient
    } ;

  public:
    vpMatrixException (const int code,  const char* format, ...)
    {
      this->code = code;
      va_list args;
      va_start(args, format);
      setMessage(format, args);
      va_end (args);
    }
    vpMatrixException (const int code, const std::string & msg)
      : vpException(code, msg){ ; }
    vpMatrixException (const int code)
      : vpException(code){ ; }
    // vpMatrixException() : vpException() { ;}
};

#endif
