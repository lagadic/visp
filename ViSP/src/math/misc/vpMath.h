/****************************************************************************
 *
 * $Id: vpMath.h,v 1.11 2008-07-18 09:26:08 fspindle Exp $
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
  \file vpMath.h
  \brief Provides simple Math computation that are not available in
  the C mathematics library (math.h)
*/



#ifndef vpMATH_HH
#define vpMATH_HH

#include <math.h>
#include <visp/vpConfig.h>

#ifdef WIN32	// Not defined in Microsoft math.h

# ifndef M_PI
#   define M_PI            3.14159265358979323846f
# endif

# ifndef M_PI_2
#   define M_PI_2          (M_PI/2.f)
# endif

# ifndef M_PI_4
#   define M_PI_4          (M_PI/4.f)
# endif

#endif


/*!
  \class vpMath

  \brief Provides simple Math computation that are not available in
  the C mathematics library (math.h)

  \author Eric Marchand   (Eric.Marchand@irisa.fr) Irisa / Inria Rennes
*/

class VISP_EXPORT vpMath
{
public:

  /*!
    Convert radian in degrees.

    \param rad Angle in radian.
    \return Angle converted in degrees
  */
   static inline double deg(double rad) { return (rad*180.0)/M_PI ; }

  /*!
    Convert degrees in radian.
    \param deg Angle in degrees.
    \return Angle converted in radian.
  */
   static inline double rad(double deg) { return (deg*M_PI)/180.0 ; }

  /*!
    Compute x square value.
    \return \f$ x^2 \f$.
  */
   static inline double sqr(double x) { return x*x ; }

  //  factorial of x
   static inline double fact(int x) ;

  // combinaison
   static inline double comb(int n, int p) ;

  //   round x to the nearest integer
   static inline int round(const double x) ;

  //   return the sign of x (+-1)
   static inline int sign(double x) ;


  // test if a number equals 0 (with threshold value)
   static inline bool nul(double x, double s=0.001);

  // test if two numbers ar equals (with a user defined threshold)
   static inline bool equal(double x, double y, double s=0.001);


  /*!
    Find the maximum between two numbers (or other)
    \param a first number
    \param b second number
    \return The maximum of the two numbers
  */
  template <class Type> static Type maximum(const Type& a, const Type& b)
  {
	return (a > b) ? a : b;
  }

  /*!
    Find the minimum between two numbers (or other)
    \param a first number
    \param b second number
    \return The minimum of the two numbers
  */
  template <class Type> static Type minimum(const Type& a, const Type& b)
  {
	return (a < b) ? a : b;
  }

  /*!
    Find the absolute value of a number (or other)
    \param x  the number
    \return The absolute value of x
  */
  template <class Type> static Type abs(const Type& x)
  {
	return (x < 0) ? -x : x;
  }


  // sinus cardinal
  static inline double sinc(double x) ;
  static inline double sinc(double sinx, double x) ;
  static inline double mcosc(double cosx, double x) ;
  static inline double msinc(double sinx, double x) ;

  /*!
    Exchange two numbers.

    \param a First number to exchange.
    \param b Second number to exchange
  */
  template <class Type> static void swap(Type& a, Type& b)
  {
	Type tmp = b;
        b = a;
	a = tmp;
  }

private:
  static const double ang_min_sinc;
  static const double ang_min_mc;
};



//Begining of the inline functions definition


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

  \param x : Value to round.

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
  Compares  \f$ | x | \f$ to \f$ s \f$.
  \param x : Value to test.
  \param s : Tolerance threshold
  \return true if \f$ | x | < s \f$.

*/
bool vpMath::nul(double x, double s)
{
  return(fabs(x)<s);
}

/*!
  Compares  \f$ | x - y | \f$ to \f$ s \f$.
  \param x : x value.
  \param y : y value.
  \param s : Tolerance threshold
  \return true if \f$ | x - y | < s \f$.
*/
bool vpMath::equal(double x, double y, double s)
{
  return( nul(x-y, s) );
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

  Compute sinus cardinal \f$ sinx / x \f$.

  \param sinx Value of sin(x).
  \param x Value of x.

  \return Sinus cardinal.

*/
double vpMath::sinc(double sinx, double x)
{
  if (fabs(x) < ang_min_sinc) return 1.0 ;
  else  return (sinx/x) ;
}

/*!
  Compute \f$ (1-cos(x))/x^2 \f$

  \param cosx Value of cos(x).
  \param x Value of x.

  \return \f$ (1-cosx)/x^2 \f$

*/
double vpMath::mcosc(double cosx, double x)
{
  if (fabs(x) < ang_min_mc) return 0.5 ;
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
  if (fabs(x) < ang_min_mc) return (1./6.0) ;
  else  return ((1.0-sinx/x)/x/x) ;
}




#endif


/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
