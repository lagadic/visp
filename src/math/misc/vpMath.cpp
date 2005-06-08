
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet Lagadic / IRISA-INRIA Rennes, 2005
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      vpMath.h
 * Project:   ViSP2
 * Author:    Eric Marchand
 * From:      simple-math, ViSP 1.6.8 (author:Eric Marchand)
 *
 * Version control
 * ===============
 *
 *  $Id: vpMath.cpp,v 1.1.1.1 2005-06-08 07:08:05 fspindle Exp $
 *
 * Description
 * ============
 *  provide some simple mathematical function that are not available in
 *  the C mathematics library (math.h)
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

/*!
  \file vpMath.cpp
  \brief Provides simple Math computation that are not available in
  the C mathematics library (math.h)

  \author Eric Marchand   (Eric.Marchand@irisa.fr) Irisa / Inria Rennes
*/


#include <math.h>
#include <vpMath.h>

/*
  \class vpMath

  \brief Provides simple Math computation that are not available in
  the C mathematics library (math.h)

  \author Eric Marchand   (Eric.Marchand@irisa.fr) Irisa / Inria Rennes
*/

//!  factorial of x
long double vpMath::fact(int x)
{
    if ( (x == 1) || (x == 0)) return 1;
    return x * fact(x-1);
}

/*! calcul de combinaison
 */
long double vpMath::comb(int n, int p)
{
    if (n == p) return 1;
    return fact(n)/ (fact(n-p) * fact(p));
}


//! x power ex
double vpMath::exp(double x, int ex)
{
    return pow(x, ex);
}

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

int vpMath::sign(double x)
{
    if (fabs(x) < 1e-15) return 0 ;else
    {
	if (x<0) return -1 ; else return 1 ;
    }
}

double vpMath::max(const double x, const double y)
{
    if (x>y) return x ; else return y ;
}

double vpMath::min(const double x, const double y)
{
    if (x>y) return y ; else return x ;
}

int vpMath::max(const int x, const int y)
{
    if (x>y) return x ; else return y ;
}

int vpMath::min(const int x, const int y)
{
    if (x>y) return y ; else return x ;
}

double vpMath::sinc(double x)
{
  if (fabs(x) < 1e-6) return 1.0 ;
  else  return sin(x)/x ;
}

void vpMath::swap(double &a, double &b)
{
  double tmp = a ;
  a = b ;
  b = tmp ;
}

void vpMath::swap(int &a, int &b)
{
  int tmp = a ;
  a = b ;
  b = tmp ;
}

/*!
  \brief Calcul de l'arctangente atan(y/x)
*/

double
vpMath::atan2(double y,double x)
{
  double xx,yy,zz;
  xx = fabs(x);yy = fabs(y);

  if (xx > 0.0001) zz = atan(yy/xx);
  else zz = M_PI/2;

  if(y >= 0.0)
  {
    if(x >=0.0) zz = zz;
    else zz = M_PI - zz;
  }
  else
  {
    if(x >=0.0) zz =  - zz;
    else zz = zz - M_PI;
  }
  return(zz);
}

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
