
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet Lagadic / IRISA-INRIA Rennes, 2005
 * www  : http://www.irisa.fr/lagadic
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      vpTime.h
 * Project:   ViSP2
 * Author:    Eric Marchand
 *
 * Version control
 * ===============
 *
 *  $Id: vpTime.h,v 1.5 2006-04-19 09:01:22 fspindle Exp $
 *
 * Description
 * ============
 * time management and measurement
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/


#ifndef TIME_HH
#define TIME_HH

/*!
  \file vpTime.h
  \brief Time management and measurement

*/

/*!
  \class vpTime
  \brief Time management and measurement

*/


class vpTime
{
public:
  static double minTimeForUsleepCall;

  static double measureTimeSecond() ;
  static double measureTimeMs() ;
  static double measureTimeMicros() ;
  static int  wait(double t0, double t) ;
  static void wait(double t) ;
} ;

#endif
