
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet Lagadic / IRISA-INRIA Rennes, 2005
 * www  : http://www.irisa.fr/lagadic
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      vpMatrixException.h
 * Project:   ViSP2
 * Author:    Eric Marchand
 *
 * Version control
 * ===============
 *
 *  $Id: vpTime.h,v 1.1.1.1 2005-06-08 07:08:11 fspindle Exp $
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
  \brief time management and measurement
  \author Francois Chaumette
  \date in 19... a long time ago...

*/

/*!
  \class vpTime
  \brief time management and measurement
  \author Francois Chaumette
  \date in 19... a long time ago...

*/

class vpTime
{
private:
  static long measureTime() ;
public:
  //! mesaure the time in seconds since the last call to the function
  static double measureTimeSecond() ;
  //! mesaure the time in miliseconds since the last call to the function
  static int measureTimeMs() ;
  //! mesaure the time in microseconds since the last call to the function
  static long measureTimeMicros() ;
  static int wait(int t0,int t) ;
  //! wait t milisecond
  static void wait(int t) ;
} ;

#endif
