/****************************************************************************
 *
 * $Id$
 *
 * Copyright (C) 1998-2010 Inria. All rights reserved.
 *
 * This software was developed at:
 * IRISA/INRIA Rennes
 * Projet Lagadic
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * http://www.irisa.fr/lagadic
 *
 * This file is part of the ViSP toolkit.
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



