/*
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2023 by Inria. All rights reserved.
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
 * See https://visp.inria.fr for more information.
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
 * Exception that can be emitted by the vpServo class and its derivatives.
 */

/*!
 * \file vpServoException.h
 * \brief error that can be emitted by the vpServo class and its derivatives
 */

#ifndef _vpServoException_h_
#define _vpServoException_h_

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpException.h>

#include <iostream>
#include <string>

BEGIN_VISP_NAMESPACE
/*!
 * \class vpServoException
 * \brief Error that can be emitted by the vpServo class and its derivatives.
*/
class VISP_EXPORT vpServoException : public vpException
{
public:
  /*!
   * \brief Lists the possible error than can be emitted while calling
   * vpServo member
   */
  enum errorServoCodeEnum
  {
    //! Current or desired feature list is empty
    noFeatureError,
    //! No degree of freedom is available to achieve the secondary task.
    noDofFree,
    //! Task was not killed properly
    notKilledProperly,
    //! Other exception
    servoError
  };

public:
  /*!
   * Constructor.
   */
  vpServoException(int id, const char *format, ...)
  {
    this->code = id;
    va_list args;
    va_start(args, format);
    setMessage(format, args);
    va_end(args);
  }

  /*!
   * Constructor.
   */
  vpServoException(int id, const std::string &msg) : vpException(id, msg) { }

  /*!
   * Constructor.
   */
  VP_EXPLICIT vpServoException(int id) : vpException(id) { }
};
END_VISP_NAMESPACE
#endif
