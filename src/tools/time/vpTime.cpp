
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet Lagadic / IRISA-INRIA Rennes, 2005
 * www  : http://www.irisa.fr/lagadic
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      vpTime.cpp
 * Project:   ViSP2
 * Author:    Eric Marchand
 *
 * Version control
 * ===============
 *
 *  $Id: vpTime.cpp,v 1.8 2006-04-19 09:01:22 fspindle Exp $
 *
 * Description
 * ============
 * time management and measurement
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

#include <iostream>
#include <visp/vpTime.h>
#include <visp/vpDebug.h>

using namespace std;

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
#  include <mmsystem.h>
#  include <winbase.h>
#endif

double vpTime::minTimeForUsleepCall = 20; /*! Threshold to activate usleep() in
					    waiting methods. This threshold is
					    needed, because usleep() is not
					    accurate on many machines. */


/*!
  Return the time in milliseconds.

  \sa measureTimeMicros(), measureTimeSecond()
*/
double
vpTime::measureTimeMs()
{
#if defined WIN32
  return(timeGetTime());
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
  return(1000.0 * timeGetTime());
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
      Sleep((long )(timeToWait-vpTime::minTimeForUsleepCall));
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



