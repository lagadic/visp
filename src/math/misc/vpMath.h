
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
 *  $Id: vpMath.h,v 1.1.1.1 2005-06-08 07:08:05 fspindle Exp $
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
  //! convert radian in degre
    static double deg(double rad) { return (rad*180.0)/M_PI ; }

  //! convert degre in radian
   static double rad(double deg) { return (deg*M_PI)/180.0 ; }

  //! compute square
   static double sqr(double x) { return x*x ; }

  //!  factorial of x
   static long double fact(int x) ;

  //! calcul de combinaison
   static long double comb(int n, int p) ;

  //! exponential computation
   static  double exp(double x, int ex) ;

  //!   round x to the nearest integer
  static int round(const double x) ;

  //!   return the sign of x (+-1)
   static int sign(double x) ;

  //! max of two numbers
   static  double max(const double x,const  double y) ;
  //! min of two numbers
   static double min(const double x, const double y) ;

  //! max of two numbers
   static  int max(const int x, const int y) ;
  //! min of two numbers
   static int min(const int x,const  int y) ;

  //! sinus cardinal
   static double sinc(double x) ;

   static void swap(double &a, double &b) ;
   static void swap(int &a, int &b) ;

  //! Calcul de l'arctangente atan(y/x)
  static double atan2(double y,double x) ;



};
#endif


/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
