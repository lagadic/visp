/****************************************************************************
 *
 * $Id$
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2011 by INRIA. All rights reserved.
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
 * Time management and measurement.
 *
 * Authors:
 * Eric Marchand
 * Fabien Spindler
 *
 *****************************************************************************/


#include <iostream>
#include <visp/vpTime.h>
#include <visp/vpDebug.h>



/*!
  \file vpTime.cpp
  \brief Time management and measurement

*/


// Unix depend version

#if defined UNIX
#  include <sys/time.h>
#  include <unistd.h>
#elif defined WIN32
#  include <windows.h>
#  include <winbase.h>
#endif

double vpTime::minTimeForUsleepCall = 4; /*! This time is in
					    milli-seconds. Threshold to
					    activate usleep() in waiting
					    methods (see wait()). This
					    threshold is needed, because
					    usleep() is not accurate on many
					    machines.  Call sleep() functions
					    during the time to wait minus
					    vpTime::minTimeForUsleepCall. The
					    rest of the time to wait is managed
					    by a while loop. */


/*!
  Return the time in milliseconds.

  \sa measureTimeMicros(), measureTimeSecond()
*/
double
vpTime::measureTimeMs()
{
#if defined WIN32
  LARGE_INTEGER time, frequency;
  QueryPerformanceFrequency(&frequency);
  if(frequency.QuadPart == 0){
    return(timeGetTime());
  }
  else{
  QueryPerformanceCounter(&time);
  return (double)(1000.0*time.QuadPart/frequency.QuadPart);
  }
#elif defined UNIX
  struct timeval tp;
  gettimeofday(&tp,0);
  return(1000.0*tp.tv_sec + tp.tv_usec/1000.0);
#endif
}


/*!
  Return the time in microseconds.

  \sa measureTimeMs(), measureTimeSecond()
*/
double
vpTime::measureTimeMicros()
{
#ifdef WIN32
  LARGE_INTEGER time, frequency;
  QueryPerformanceFrequency(&frequency);
  if(frequency.QuadPart == 0){
    return(timeGetTime());
  }
  else{
  QueryPerformanceCounter(&time);
  return (double)(1000000.0*time.QuadPart/frequency.QuadPart);
  }
#else

  struct timeval tp;
  gettimeofday(&tp,0);
  return(1000000.0*tp.tv_sec + tp.tv_usec);
#endif
}



/*!

  Wait t miliseconds after t0 (in ms).

  The waiting is done by a call to usleep() if the time to wait is greater than
  vpTime::minTimeForUsleepCall.

  \param t0 : reference time (in ms)
  \param t  : time to wait (in ms)

  \return 0 : The function did wait.
  \return 1 : The time was already over, no need to wait.
*/

int
vpTime::wait(double t0, double t)
{
  double timeCurrent, timeToWait;
  timeCurrent = measureTimeMs();

  timeToWait = t0 + t - timeCurrent;

  if ( timeToWait <= 0. ) // no need to wait
    return(1);
  else {
#if defined UNIX
    if (timeToWait > vpTime::minTimeForUsleepCall) {
      usleep((unsigned long )((timeToWait-vpTime::minTimeForUsleepCall)*1000));
    }
#elif defined WIN32
    if (timeToWait > vpTime::minTimeForUsleepCall) {
      Sleep((long)(timeToWait-vpTime::minTimeForUsleepCall));
    }
#endif
    // Blocking loop to have an accurate waiting
    do {
      timeCurrent = measureTimeMs();
      timeToWait = t0 + t - timeCurrent;

    } while (timeToWait > 0.);

    return 0;
  }
}


/*!
  Wait t miliseconds from now.

  The waiting is done by a call to usleep() if the time to wait is greater than
  vpTime::minTimeForUsleepCall.

  \param t : Time to wait in ms.

*/
void vpTime::wait(double t)
{
  double t0, timeCurrent, timeToWait;
  t0 = timeCurrent = measureTimeMs();

  timeToWait = t;

  if ( timeToWait <= 0. ) // no need to wait
    return;
  else {
#if defined UNIX
	if (timeToWait > vpTime::minTimeForUsleepCall) {
      usleep((unsigned long )((timeToWait-vpTime::minTimeForUsleepCall)*1000));
    }
#elif defined WIN32
    if (timeToWait > vpTime::minTimeForUsleepCall) {
      Sleep((long )(timeToWait-vpTime::minTimeForUsleepCall));
    }
#endif
    // Blocking loop to have an accurate waiting
    do {
      timeCurrent = measureTimeMs();
      timeToWait = t0 + t - timeCurrent;

    } while (timeToWait > 0.);

    return;
  }
}

/*!

  Return the measured time in seconds.

  \sa measureTimeMs()
*/
double  vpTime::measureTimeSecond()
{
  return vpTime::measureTimeMs()/1000.0 ;
}



