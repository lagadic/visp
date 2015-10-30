/****************************************************************************
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2015 by Inria. All rights reserved.
 *
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ViSP with software that can not be combined with the GNU
 * GPL, please contact Inria about acquiring a ViSP Professional
 * Edition License.
 *
 * See http://visp.inria.fr for more information.
 *
 * This software was developed at:
 * Inria Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 *
 * If you have questions regarding the use of this file, please contact
 * Inria at visp@inria.fr
 *
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Description:
 * Time management and measurement.
 *
 * Authors:
 * Eric Marchand
 * Fabien Spindler
 *
 *****************************************************************************/



#include <visp3/core/vpTime.h>
#include <visp3/core/vpDebug.h>
#include <iostream>


/*!
  \file vpTime.cpp
  \brief Time management and measurement

*/


// Unix depend version

#if !defined(_WIN32) && (defined(__unix__) || defined(__unix) || (defined(__APPLE__) && defined(__MACH__))) // UNIX
#  include <sys/time.h>
#  include <unistd.h>
#elif defined(_WIN32)
#  include <windows.h>
#  include <winbase.h>
#endif

#ifndef DOXYGEN_SHOULD_SKIP_THIS
namespace vpTime
{
#endif

/*!
   This time is in milli-seconds. Threshold to activate usleep() in waiting
   methods (see wait()). This threshold is needed, because usleep() is not
   accurate on many machines.  Call sleep() functions during the time to wait
   minus vpTime::minTimeForUsleepCall. The rest of the time to wait is managed
   by a while loop.
*/
double minTimeForUsleepCall = 4;

/*!
   \return The time during which a while loop is used to handle the time
   wainting functions. When the time to wait is greater than that value, we use
   non blocking functions like usleep() instead.
 */
double getMinTimeForUsleepCall()
{
  return minTimeForUsleepCall;
}

/*!
  \fn vpTime::measureTimeMs()
  Return the time in milliseconds since January 1st 1970.

  \sa measureTimeMicros(), measureTimeSecond()
*/
double measureTimeMs()
{
#if defined(_WIN32)
  LARGE_INTEGER time, frequency;
  QueryPerformanceFrequency(&frequency);
  if(frequency.QuadPart == 0){
    return(timeGetTime());
  }
  else{
    QueryPerformanceCounter(&time);
    return (double)(1000.0*time.QuadPart/frequency.QuadPart);
  }
#elif !defined(_WIN32) && (defined(__unix__) || defined(__unix) || (defined(__APPLE__) && defined(__MACH__))) // UNIX
  struct timeval tp;
  gettimeofday(&tp,0);
  return(1000.0*tp.tv_sec + tp.tv_usec/1000.0);
#endif
}

/*!
  Return the time in microseconds since January 1st 1970.

  \sa measureTimeMs(), measureTimeSecond()
*/
double measureTimeMicros()
{
#if defined(_WIN32)
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
int wait(double t0, double t)
{
  double timeCurrent, timeToWait;
  timeCurrent = measureTimeMs();

  timeToWait = t0 + t - timeCurrent;

  if ( timeToWait <= 0. ) // no need to wait
    return(1);
  else {
#if !defined(_WIN32) && (defined(__unix__) || defined(__unix) || (defined(__APPLE__) && defined(__MACH__))) // UNIX
    if (timeToWait > vpTime::minTimeForUsleepCall) {
      usleep((unsigned long )((timeToWait-vpTime::minTimeForUsleepCall)*1000));
    }
#elif defined(_WIN32)
    if (timeToWait > vpTime::minTimeForUsleepCall) {
      Sleep((DWORD)(timeToWait-vpTime::minTimeForUsleepCall));
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
void wait(double t)
{
  double t0, timeCurrent, timeToWait;
  t0 = timeCurrent = measureTimeMs();

  timeToWait = t;

  if ( timeToWait <= 0. ) // no need to wait
    return;
  else {
#if !defined(_WIN32) && (defined(__unix__) || defined(__unix) || (defined(__APPLE__) && defined(__MACH__))) // UNIX
    if (timeToWait > vpTime::minTimeForUsleepCall) {
      usleep((unsigned long )((timeToWait-vpTime::minTimeForUsleepCall)*1000));
    }
#elif defined(_WIN32)
    if (timeToWait > vpTime::minTimeForUsleepCall) {
      Sleep((DWORD)(timeToWait-vpTime::minTimeForUsleepCall));
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

  Return the measured time in seconds since January 1st 1970.

  \sa measureTimeMs()
*/
double  measureTimeSecond()
{
  return vpTime::measureTimeMs()/1000.0 ;
}

/*!
  Sleep t miliseconds from now.

  \param t : Time to sleep in ms.

*/
void sleepMs(double t)
{
#if !defined(_WIN32) && (defined(__unix__) || defined(__unix) || (defined(__APPLE__) && defined(__MACH__))) // UNIX
  usleep((unsigned long )(t*1000));
#elif defined(_WIN32)
  Sleep((DWORD)(t));
#endif
}

#ifndef DOXYGEN_SHOULD_SKIP_THIS
};
#endif
