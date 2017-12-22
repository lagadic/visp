/****************************************************************************
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2017 by Inria. All rights reserved.
 *
 * This software is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ViSP with software that can not be combined with the GNU
 * GPL, please contact Inria about acquiring a ViSP Professional
 * Edition License.
 *
 * See http://visp.inria.fr for more information.
 *
 * This software was developed at:
 * Inria Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 *
 * If you have questions regarding the use of this file, please contact
 * Inria at visp@inria.fr
 *
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Description:
 * Exception that can be emited by the vpFeature class and its derivates.
 *
 * Authors:
 * Eric Marchand
 *
 *****************************************************************************/

#ifndef __vpFeatureException_H
#define __vpFeatureException_H

/* -------------------------------------------------------------------------
 */
/* --- INCLUDE -------------------------------------------------------------
 */
/* -------------------------------------------------------------------------
 */

/* \file vpFeatureException.h
   \brief error that can be emited by the vpFeature class and its derivates
 */
/* Classes standards. */

#include <iostream> /* Classe std::ostream.    */
#include <string>   /* Classe string.     */
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpException.h>

/* -------------------------------------------------------------------------
 */
/* --- CLASS ---------------------------------------------------------------
 */
/* -------------------------------------------------------------------------
 */

/*!
  \class vpFeatureException
  \ingroup group_visual_features
  \brief Error that can be emited by the vpBasicFeature class and its
  derivates.
 */
class VISP_EXPORT vpFeatureException : public vpException
{
public:
  /*!
  \brief Lists the possible error than can be emmited while calling
  vpFeature member
 */
  enum errorFeatureCodeEnum {
    //! feature list or desired feature list is empty
    badErrorVectorError,
    sizeMismatchError,
    notInitializedError,
    badInitializationError
  };

public:
  vpFeatureException(const int id, const char *format, ...)
  {
    this->code = id;
    va_list args;
    va_start(args, format);
    setMessage(format, args);
    va_end(args);
  }
  vpFeatureException(const int id, const std::string &msg) : vpException(id, msg) { ; }
  explicit vpFeatureException(const int id) : vpException(id) { ; }
};

#endif
