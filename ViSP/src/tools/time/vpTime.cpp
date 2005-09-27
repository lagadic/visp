
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
 *  $Id: vpTime.cpp,v 1.3 2005-09-27 12:31:07 fspindle Exp $
 *
 * Description
 * ============
 * time management and measurement
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/


#include <visp/vpTime.h>


/*!
  \file vpTime.h
  \brief time management and measurement
  \author Francois Chaumette
  \date in 19... a long time ago...

*/

#ifdef WIN32
// Partie dependante de la plateforme (M$ Windows)

#include <afx.h>
#include <sys/types.h>
#include <sys/timeb.h>
#include <app/CViSPApp.h>
#include <unistd.h>

/*!
  \ingroup libtime
  \brief Return the time in milli seconds
*/

int MeasureTimeMs()
{
	LARGE_INTEGER count;
	QueryPerformanceCounter(&count);
	return (int) (count.QuadPart*1000/(double)CViSPApp::getFrequency());
}

/*!
  \ingroup libtime
  \brief Wait t miliseconds after t0 ms

  \param t0 : reference time (in ms)
  \param t  : time to wait (in ms)

  \return 1 the function did wait, 0 the time was already over
*/

int Wait(int t0, int t)
{

	if (MeasureTimeMs()>t0+t)
		return 1;
	Wait(t-(MeasureTimeMs()-t0));
	return 0;
}


/*!
  \ingroup libtime
  \brief wait t  miliseconds from now
  \param t time to wait
  \return 1 the function did wait, 0 the time was already over
*/

void Wait(int t)
{
	int tfin=MeasureTimeMs()+t;
	while(MeasureTimeMs()<tfin);
}


#endif
#if (defined(__Linux_) || defined(__SunOS_))



// Unix depend version


#include <sys/time.h>
#include <unistd.h>

long
vpTime::measureTime()
{
  struct timeval tp;

  gettimeofday(&tp,0);
  return(tp.tv_usec);
}


/*!
  \brief Return the time in  milliseconds
*/
double
vpTime::measureTimeMs()
{
   struct timeval tp;
   gettimeofday(&tp,0);
   return(1000.0*tp.tv_sec + tp.tv_usec/1000.0);
}


/*!
  \brief Return the time in  milliseconds
*/
double
vpTime::measureTimeMicros()
{
   struct timeval tp;
   gettimeofday(&tp,0);
   return(1000000.0*tp.tv_sec + tp.tv_usec);
}



/*!
  \brief Wait t miliseconds after t0 ms

  \param t0 : reference time (in ms)
  \param t  : time to wait (in ms)

  \return 1 the function did wait, 0 the time was already over
*/

int
vpTime::wait(double t0, double t)
{
  struct timeval tp;

  gettimeofday(&tp, 0);
  if ((1000.0*tp.tv_sec + tp.tv_usec/1000.0 - t0) > t) return(1);
  else
    {
      do
	{
	  gettimeofday(&tp, 0);
	}
      while ((1000.0*tp.tv_sec + tp.tv_usec/1000.0 - t0) < t);

 return(0);
    }
}


/*!
  \brief wait t  miliseconds from now
  \param t time to wait
  \return 1 the function did wait, 0 the time was already over
*/
void vpTime::wait(int t)
{

  usleep(t*1000) ;
  //  wait(measureTimeMs(),t);
}



#endif // Partie dependante de la plateforme



/*!
  \ingroup libtime
  \brief Return the time in  seconds
*/
double  vpTime::measureTimeSecond()
{
  return vpTime::measureTimeMs()/1000.0 ;
}



