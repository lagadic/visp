
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet Lagadic / IRISA-INRIA Rennes, 2005
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      vpMath.h
 * Project:   ViSP 2.0
 * Author:    Eric Marchand
 * From:      simple-math, ViSP 1.6.8 (author:Eric Marchand)
 *
 * Version control
 * ===============
 *
 *  $Id: vpMath.h,v 1.4 2006-04-19 09:01:21 fspindle Exp $
 *
 * Description
 * ============
 *  provide some simple mathematical function that are not available in
 *  the C mathematics library (math.h)
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

/*!
  \file vpMath.h
  \brief Provides simple Math computation that are not available in
  the C mathematics library (math.h)
*/



#ifndef vpMATH_HH
#define vpMATH_HH

#include <math.h>

#ifdef WIN32	// Non defines in M$ math.h
#define M_PI            3.14159265358979323846
#define M_PI_2          1.57079632679489661923
#define M_PI_4          0.78539816339744830962
#endif


/*
  \class vpMath

  \brief Provides simple Math computation that are not available in
  the C mathematics library (math.h)

  \author Eric Marchand   (Eric.Marchand@irisa.fr) Irisa / Inria Rennes
*/
class vpMath
{

public:
  /*!
    Convert radian in degrees.

    \param rad Angle in radian.
    \return Angle converted in degrees
  */
  static double deg(double rad) { return (rad*180.0)/M_PI ; }

  /*!
    Convert degrees in radian.
    \param deg Angle in degrees.
    \return Angle converted in radian.
  */

  static double rad(double deg) { return (deg*M_PI)/180.0 ; }

  /*!
    Compute x square value.
    \return \f$ x^2 \f$.
  */
  static double sqr(double x) { return x*x ; }

  //  factorial of x
  static double fact(int x) ;

  // combinaison
  static double comb(int n, int p) ;

  //   round x to the nearest integer
  static int round(const double x) ;

  //   return the sign of x (+-1)
  static int sign(double x) ;

  // max of two numbers
  static  double max(const double x,const  double y) ;
  // min of two numbers
  static double min(const double x, const double y) ;

  // max of two numbers
  static  int max(const int x, const int y) ;
  // min of two numbers
  static int min(const int x,const  int y) ;

  // sinus cardinal
  static double sinc(double x) ;
  static double sinc(double sinx, double x) ;
  static double mcosc(double cosx, double x) ;
  static double msinc(double sinx, double x) ;

  static void swap(double &a, double &b) ;
  static void swap(int &a, int &b) ;
};
#endif


/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
