/****************************************************************************
 *
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2019 by Inria. All rights reserved.
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
 * Print CPU features.
 *
 *****************************************************************************/

#include <cstdlib>
#include <iostream>
#include <visp3/core/vpCPUFeatures.h>

#if defined __SSE2__ || defined _M_X64 || (defined _M_IX86_FP && _M_IX86_FP >= 2)
#include <emmintrin.h>
#define VISP_HAVE_SSE2 1

#if defined __SSE3__ || (defined _MSC_VER && _MSC_VER >= 1500)
#include <pmmintrin.h>
#define VISP_HAVE_SSE3 1
#else
#define VISP_HAVE_SSE3 0
#endif
#if defined __SSSE3__ || (defined _MSC_VER && _MSC_VER >= 1500)
#include <tmmintrin.h>
#define VISP_HAVE_SSSE3 1
#else
#define VISP_HAVE_SSSE3 0
#endif
#else
#define VISP_HAVE_SSE2 0
#define VISP_HAVE_SSE3 0
#define VISP_HAVE_SSSE3 0
#endif

#define VALUE_TO_STRING(x) #x
#define VALUE(x) VALUE_TO_STRING(x)

int main()
{
  vpCPUFeatures::printCPUInfo();
  std::cout << "checkSSE2: " << vpCPUFeatures::checkSSE2() << " ; VISP_HAVE_SSE2: " << VALUE(VISP_HAVE_SSE2)
            << std::endl;
  std::cout << "checkSSE3: " << vpCPUFeatures::checkSSE3() << " ; VISP_HAVE_SSE3: " << VALUE(VISP_HAVE_SSE3)
            << std::endl;
  std::cout << "checkSSSE3: " << vpCPUFeatures::checkSSSE3() << " ; VISP_HAVE_SSSE3: " << VALUE(VISP_HAVE_SSSE3)
            << std::endl;

  return EXIT_SUCCESS;
}
