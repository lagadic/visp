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
#include <visp3/core/vpMath.h>

#if defined(VISP_HAVE_FUNC_ISNAN) || defined(VISP_HAVE_FUNC_STD_ISNAN)
#  include <cmath>
#elif defined(VISP_HAVE_FUNC__ISNAN)
#  include <float.h>
#else
#  if defined _MSC_VER || defined __BORLANDC__
typedef __int64 int64;
typedef unsigned __int64 uint64;
#  else
typedef int64_t int64;
typedef uint64_t uint64;
#  endif

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
  return (_isnan(value) != 0);
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

/*!
  Compute \f$ (1-cos(x))/x^2 \f$

  \param cosx : Value of cos(x).
  \param x : Value of x.

  \return \f$ (1-cosx)/x^2 \f$

*/
double vpMath::mcosc(double cosx, double x)
{
  if (fabs(x) < ang_min_mc) return 0.5 ;
  else  return ((1.0-cosx)/x/x) ;
}

/*!

  Compute \f$ (1-sinc(x))/x^2 \f$ with \f$ sinc(x) = sinx / x \f$.

  \param sinx : value of sin(x).
  \param x  : Value of x.

  \return \f$ (1-sinc(x))/x^2 \f$

*/
double vpMath::msinc(double sinx, double x)
{
  if (fabs(x) < ang_min_mc) return (1./6.0) ;
  else  return ((1.0-sinx/x)/x/x) ;
}

/*!

  Compute sinus cardinal \f$ \frac{sin(x)}{x} \f$.

  \param x : Value of x.

  \return Sinus cardinal.

*/
double vpMath::sinc(double x)
{
  if (fabs(x) < ang_min_sinc) return 1.0 ;
  else  return sin(x)/x ;
}
/*!

  Compute sinus cardinal \f$ \frac{sin(x)}{x}\f$.

  \param sinx : Value of sin(x).
  \param x : Value of x.

  \return Sinus cardinal.

*/
double vpMath::sinc(double sinx, double x)
{
  if (fabs(x) < ang_min_sinc) return 1.0 ;
  else  return (sinx/x) ;
}
