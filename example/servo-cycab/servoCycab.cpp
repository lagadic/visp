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
 * This file is part of the ViSP toolkit
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
 * Send a command to the car-like Cycab mobile robot.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/
/*!
  \example servoCycab.cpp

  Send a command to the car-like Cycab mobile robot.

*/

#include <iostream>

#include <signal.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <math.h>
#include <sys/time.h>

#include <visp/vpConfig.h>

using namespace std;

#ifdef VISP_HAVE_CYCAB
#include <visp/vpRobotCycab.h>
#include <visp/vpMath.h>


vpRobotCycab *cycab = NULL;
bool bFinish = false;

bool end = false;

// The first CTRL-C stop properly the car, the second CTRL-C ends the execution
void sighdl(int n)
{
  if(cycab != NULL) {
    cycab->setCommand(0, 0);
    bFinish=true;
  }
  static time_t t = 0;
  printf("Received signal %d\n",n);
  if (!end) 
    t = time(NULL);
  else if ((time(NULL) - t) > 3)
    kill(getpid(),SIGKILL);
  end = true;	
}

#define MAXV     .5 // velocity in m/S
#define MAXPHI 20.0 // front steering angle in deg

int main()
{
  
  double v, phi; // Command to send
  double vm, phim; // Measures
  int kv,kphi;

  cycab = new vpRobotCycab();

  kv = kphi = 1;
  v = 0.0;
  phi = 0.0;
  signal(SIGINT,sighdl);
  signal(SIGTERM,sighdl);
  signal(SIGPIPE,sighdl);

  struct timeval tp; struct timezone tz;
  gettimeofday(&tp,&tz);
  double t0 = (1000.0*tp.tv_sec + tp.tv_usec/1000.0);		
  double t1;
  bool ctrc = false;
  double timestamp;

  while (!end) {
    // Measures the velocity and the front steering angle from odometry
    cycab->getCommand(vm, phim, timestamp); // measured values from odometry
      
    printf("State %f : v %f m/s and phi %f deg\n\t", timestamp, vm, vpMath::deg(phim));

    // Compute the command to apply to the car
    if (1) {
      v+=kv*0.002;if (fabs(v)>=MAXV) kv = -1 * kv;
      phi+=kphi*0.002;if (fabs(phi)>= vpMath::rad(MAXPHI)) kphi = -1 * kphi;
    }
    else {
      v=0.1;phi=0;
    }

    // Check is CTRL-C is requested
    if (bFinish) {
      // we stop the Cycab
      v = 0;
      phi = 0;
    }

    // Send the command
    printf("Send : v %f m/s and phi %f deg\n", v, vpMath::deg(phi));
    cycab->setCommand(v, phi);
            
    usleep(10000);

    gettimeofday(&tp,&tz);
    t1=(1000.0*tp.tv_sec + tp.tv_usec/1000.0);		
    if((t1>t0+ 1000*10)&(ctrc)) break;
    if (end && (!ctrc)) { end = false; t0=t1; ctrc=true;} 
  }
  return 0;
}
		
#else // VISP_HAVE_CYCAB		
int main()
{
  cout << "Sorry no acces to the cycab" << endl;
}
#endif //VISP_HAVE_CYCAB

