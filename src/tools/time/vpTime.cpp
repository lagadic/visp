
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
 *  $Id: vpTime.cpp,v 1.7 2006-04-10 13:53:30 fspindle Exp $
 *
 * Description
 * ============
 * time management and measurement
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

#include <iostream>
#include <visp/vpTime.h>

using namespace std;

/*!
  \file vpTime.cpp
  \brief Time management and measurement

*/

#ifdef WIN32
// MS Windows

#  include <afx.h>
#  include <sys/types.h>
#  include <sys/timeb.h>
#  include <app/CViSPApp.h>
#  include <unistd.h>

#else // UNIX

// Unix depend version

#include <sys/time.h>
#include <unistd.h>


double vpTime::minTimeForUsleepCall = 20; /*! Threshold to activate usleep() in
					    waiting methods. This threshold is
					    needed, because usleep() is not
					    accurate on many machines. */

#endif



/*!
  Return the time in milliseconds.

  \sa measureTimeMicros(), measureTimeSecond()
*/
double
vpTime::measureTimeMs()
{
#ifdef WIN32
  LARGE_INTEGER count;
  QueryPerformanceCounter(&count);
  return (int) (count.QuadPart*1000/(double)CViSPApp::getFrequency());
#else
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
   struct timeval tp;
   gettimeofday(&tp,0);
   return(1000000.0*tp.tv_sec + tp.tv_usec);
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
    if (timeToWait > vpTime::minTimeForUsleepCall) {
      usleep((unsigned long )((timeToWait-vpTime::minTimeForUsleepCall)*1000));
    }

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
    if (timeToWait > vpTime::minTimeForUsleepCall) {
      usleep((unsigned long )((timeToWait-vpTime::minTimeForUsleepCall)*1000));
    }

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



