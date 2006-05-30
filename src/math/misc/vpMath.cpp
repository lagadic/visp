/****************************************************************************
 *
 * $Id: vpMath.cpp,v 1.6 2006-05-30 08:40:43 fspindle Exp $
 *
 * Copyright (C) 1998-2006 Inria. All rights reserved.
 *
 * This software was developed at:
 * IRISA/INRIA Rennes
 * Projet Lagadic
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * http://www.irisa.fr/lagadic
 *
 * This file is part of the ViSP toolkit
 *
 * This file may be distributed under the terms of the Q Public License
 * as defined by Trolltech AS of Norway and appearing in the file
 * LICENSE included in the packaging of this file.
 *
 * Licensees holding valid ViSP Professional Edition licenses may
 * use this file in accordance with the ViSP Commercial License
 * Agreement provided with the Software.
 *
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Contact visp@irisa.fr if any conditions of this licensing are
 * not clear to you.
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

  \author Eric Marchand   (Eric.Marchand@irisa.fr) Irisa / Inria Rennes
*/


#include <math.h>
#include <visp/vpMath.h>

/*
  \class vpMath

  \brief Provides simple math computation that are not available in
  the C mathematics library (math.h)

  \author Eric Marchand   (Eric.Marchand@irisa.fr) Irisa / Inria Rennes
*/

/*!
  Compute factorial.

  \return !x
*/
double vpMath::fact(int x)
{
    if ( (x == 1) || (x == 0)) return 1;
    return x * fact(x-1);
}

/*!
  Combinaison computation.

  \return \f$ !n / (!(n-p) !p) \f$
 */
double vpMath::comb(int n, int p)
{
    if (n == p) return 1;
    return fact(n)/ (fact(n-p) * fact(p));
}


/*!
  Round x to the nearest integer.

  \return Nearest integer of x.

*/
int vpMath::round(const double x)
{
    if (sign(x) > 0)
    {
	if ((x-(int)x) <= 0.5) return (int)x ;
	else return (int)x+1 ;
    }
    else
    {
	if (fabs(x-(int)x) <= 0.5) return (int)x ;
	else return (int)x-1 ;
    }
}

/*!
  Return the sign of x.

  \return -1 if x is negative, +1 if positive.

*/
int vpMath::sign(double x)
{
    if (fabs(x) < 1e-15) return 0 ;else
    {
	if (x<0) return -1 ; else return 1 ;
    }
}

/*!

  Return the maximum value of x or y.

  \return max(x,y)
*/
double vpMath::max(const double x, const double y)
{
    if (x>y) return x ; else return y ;
}

/*!

  Return the minimum value of x or y.

  \return min(x,y)
*/
double vpMath::min(const double x, const double y)
{
    if (x>y) return y ; else return x ;
}

/*!

  Return the maximum value of x or y.

  \return max(x,y)
*/
int vpMath::max(const int x, const int y)
{
    if (x>y) return x ; else return y ;
}

/*!

  Return the minimum value of x or y.

  \return min(x,y)
*/
int vpMath::min(const int x, const int y)
{
    if (x>y) return y ; else return x ;
}
#ifdef ANG_MIN_SINC // used also in vpRotationMatrix.cpp and vpThetaUVector.cpp
#undef ANG_MIN_SINC
#endif
#define ANG_MIN_SINC 1e-8

/*!

  Compute sinus cardinal \f$ \frac{sin(x)}{x} \f$.

  \param x Value of x.

  \return Sinus cardinal.

*/
double vpMath::sinc(double x)
{
  if (fabs(x) < ANG_MIN_SINC) return 1.0 ;
  else  return sin(x)/x ;
}
/*!

  Compute sinus cardinal \f$ sinx / x \f$.

  \param sinx Value of sin(x).
  \param x Value of x.

  \return Sinus cardinal.

*/
double vpMath::sinc(double sinx, double x)
{
  if (fabs(x) < ANG_MIN_SINC) return 1.0 ;
  else  return (sinx/x) ;
}
#undef ANG_MIN_SINC

#ifdef ANG_MIN_MC // used also in vpRotationMatrix.cpp
#undef ANG_MIN_MC
#endif
#define ANG_MIN_MC 2.5e-4
/*!
  Compute \f$ (1-cos(x))/x^2 \f$

  \param cosx Value of cos(x).
  \param x Value of x.

  \return \f$ (1-cosx)/x^2 \f$

*/
double vpMath::mcosc(double cosx, double x)
{
  if (fabs(x) < ANG_MIN_MC) return 0.5 ;
  else  return ((1.0-cosx)/x/x) ;
}

/*!

  Compute \f$ (1-sinc(x))/x^2 \f$ with \f$ sinc(x) = sinx / x \f$.

  \param sinx value of sin(x).
  \param x  Value of x.

  \return \f$ (1-sinc(x))/x^2 \f$

*/
double vpMath::msinc(double sinx, double x)
{
  if (fabs(x) < ANG_MIN_MC) return (1./6.0) ;
  else  return ((1.0-sinx/x)/x/x) ;
}
#undef ANG_MIN_MC

/*!

  Exchange two numbers.

  \param a First number to exchange.
  \param b Second number to exchange

*/
void vpMath::swap(double &a, double &b)
{
  double tmp = a ;
  a = b ;
  b = tmp ;
}

/*!

  Exchange two numbers.

  \param a First number to exchange.
  \param b Second number to exchange


*/
void vpMath::swap(int &a, int &b)
{
  int tmp = a ;
  a = b ;
  b = tmp ;
}


/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
