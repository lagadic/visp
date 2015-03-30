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
 * Simple mathematical function not available in the C math library (math.h).
 *
 * Authors:
 * Eric Marchand
 *
 *****************************************************************************/

/*!
  \file vpMath.cpp
  \brief Provides simple Math computation that are not available in
  the C mathematics library (math.h)
*/

#include <stdint.h>
#include <visp/vpMath.h>

#if defined(VISP_HAVE_FUNC_ISNAN) || defined(VISP_HAVE_FUNC_STD_ISNAN)
  #include <cmath>
#elif defined(VISP_HAVE_FUNC__ISNAN)
  #include <float.h>
#else
  #if defined _MSC_VER || defined __BORLANDC__
     typedef __int64 int64;
     typedef unsigned __int64 uint64;
  #else
     typedef int64_t int64;
     typedef uint64_t uint64;
  #endif

   typedef union Cv64suf
   {
       int64 i;
       uint64 u;
       double f;
   }
   Cv64suf;
#endif

const double vpMath::ang_min_sinc = 1.0e-8;
const double vpMath::ang_min_mc = 2.5e-4;


/*!
   Check whether a double number is not a number (NaN) or not.
   \param value : Double number to check.
   \return Return true if value is not a number.
 */
bool vpMath::isNaN(const double value)
{
#if defined(VISP_HAVE_FUNC_ISNAN)
  return isnan(value);
#elif defined(VISP_HAVE_FUNC_STD_ISNAN)
  return std::isnan(value);
#elif defined(VISP_HAVE_FUNC__ISNAN)
  return _isnan(value);
#else
  #if 0
    //This trick should work for any compiler which claims to use IEEE floating point.
    //Do not work with g++ and -ffast-math option.
    return (value != value);
  #else
    //Taken from OpenCV source code CvIsNan()
    Cv64suf ieee754;
    ieee754.f = value;
    return (((unsigned)(ieee754.u >> 32) & 0x7fffffff) +
           ((unsigned)ieee754.u != 0) > 0x7ff00000) != 0;
  #endif
#endif
}
