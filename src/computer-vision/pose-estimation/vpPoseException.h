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
 * Error that can be emited by the vpPose class and its derivates
 *
 * Authors:
 * Eric Marchand
 *
 *****************************************************************************/


#ifndef __vpPoseException_H
#define __vpPoseException_H


/* ------------------------------------------------------------------------- */
/* --- INCLUDE ------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */


/* Classes standards. */

#include <visp/vpException.h>

#include <iostream>                /* Classe std::ostream.    */
#include <string>                  /* Classe string.     */

/* ------------------------------------------------------------------------- */
/* --- CLASS --------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */

/*!
  \class vpPoseException
  \ingroup Exception
  \brief Error that can be emited by the vpPose class and its derivates.
 */
class VISP_EXPORT vpPoseException : public vpException
{
public:
  /*!
    \brief Lists the possible error than can be emmited while calling
    vpPose member
   */
  enum errorCodeEnum
    {
      poseError,
      //! something is not initialized
      notInitializedError,
      //! function not implemented
      notImplementedERR,
      //! index out of range
      outOfRangeError,
      notEnoughPointError
    } ;

public:
  vpPoseException (const int code, const char * msg)
    : vpException(code, msg){ ; }
  vpPoseException (const int code, const std::string & msg)
    : vpException(code, msg){ ; }
  vpPoseException (const int code)
    : vpException(code){ ; }
 // vpPoseException() : vpException() { ;}
};





#endif /* #ifndef __vpPoseException_ERROR_H */


/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
