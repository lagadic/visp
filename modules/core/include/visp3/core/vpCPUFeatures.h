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
 * CPU features (hardware capabilities).
 *
 *****************************************************************************/

#ifndef __vpCPUFeatures_h_
#define __vpCPUFeatures_h_

/*!
  \file vpCPUFeatures.h
  \brief Check CPU features (hardware capabilities).
*/

#include <visp3/core/vpConfig.h>

/*!
  \ingroup group_core_cpu_features
  \brief Check CPU features (hardware capabilities).

  The example below shows how to check or get CPU capabilities.

  \code
#include <visp3/core/vpCPUFeatures.h>

int main()
{
  std::cout << "checkSSE2: " << vpCPUFeatures::checkSSE2() << std::endl;
  std::cout << "CPU info: " << vpCPUFeatures::printCPUInfo() << std::endl;
  return 0;
}
  \endcode
*/

namespace vpCPUFeatures
{
VISP_EXPORT bool checkSSE2();
VISP_EXPORT bool checkSSE3();
VISP_EXPORT bool checkSSSE3();
VISP_EXPORT bool checkSSE41();
VISP_EXPORT bool checkSSE42();
VISP_EXPORT bool checkAVX();
VISP_EXPORT bool checkAVX2();
VISP_EXPORT void printCPUInfo();
}

#endif
