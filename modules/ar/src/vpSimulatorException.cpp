/*
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2025 by Inria. All rights reserved.
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

/*! \file vpSimulatorException.h
   \brief error that can be emitted by the vpSimulator class and its derivatives
 */

#include <visp3/ar/vpSimulatorException.h>

#ifdef VISP_BUILD_DEPRECATED_FUNCTIONS
BEGIN_VISP_NAMESPACE
vpSimulatorException::vpSimulatorException(int id, const char *format, ...)
{
  this->code = id;
  va_list args;
  va_start(args, format);
  setMessage(format, args);
  va_end(args);
}

vpSimulatorException::vpSimulatorException(int id, const std::string &msg) : vpException(id, msg) { }

vpSimulatorException::vpSimulatorException(int id) : vpException(id) { }
END_VISP_NAMESPACE
#elif !defined(VISP_BUILD_SHARED_LIBS)
// Work around to avoid warning: libvisp_ar.a(vpSimulatorException.cpp.o) has no symbols
void dummy_vpSimulatorException() { }

#endif
