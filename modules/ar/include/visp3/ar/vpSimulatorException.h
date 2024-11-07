/*
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2024 by Inria. All rights reserved.
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
 * Exceptions that can be emitted by the simulator classes.
 */

/*!
 * \file vpSimulatorException.h
 *  \brief Error that can be emitted by the vpSimulator class and its derivatives
 */

#ifndef VP_SIMULATOR_EXCEPTION_H
#define VP_SIMULATOR_EXCEPTION_H

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpException.h>

#ifdef VISP_BUILD_DEPRECATED_FUNCTIONS
#include <iostream> /* Classe std::ostream.    */
#include <string>   /* Classe string.     */

BEGIN_VISP_NAMESPACE
/*!
 * \class vpSimulatorException
 * \brief Error that can be emitted by the vpSimulator class and its derivatives.
*/
class VISP_EXPORT vpSimulatorException : public vpException
{
public:
  /*!
   * Lists the possible error than can be emitted while calling
   * vpSimulator member
   */
  enum errorSimulatorCodeEnum
  {
    ioError,                       //!< I/O error
    noFileNameError,               //!< Filename error
    notInitializedError,           //!< Initialization error
    windowSizeNotInitializedError, //!< Window size not initialized
    badInitializationError         //!< Initialization error
  };

public:
  /*!
   * Constructor.
   */
  vpSimulatorException(int id, const char *format, ...);

  /*!
   * Constructor.
   */
  vpSimulatorException(int id, const std::string &msg);

  /*!
   * Constructor.
   */
  VP_EXPLICIT vpSimulatorException(int id);
};
END_VISP_NAMESPACE
#endif
#endif
