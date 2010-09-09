/****************************************************************************
 *
 * $Id$
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2010 by INRIA. All rights reserved.
 * 
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
 * See the file LICENSE.GPL at the root directory of this source
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
#include <math.h>

#include <visp/vpConfig.h>

using namespace std;

#ifdef VISP_HAVE_CYCAB
#include <visp/vpRobotCycab.h>
#include <visp/vpMath.h>
#include <visp/vpTime.h>


vpRobotCycab *cycab = NULL;
bool bFinish = false;

bool end = false;

// The first CTRL-C stop properly the car by decreasing the velocity
// and the steering angle, the second CTRL-C ends the execution
void sighdl(int n)
{
  printf("Received signal %d\n",n);
  bFinish=true;
  end = true;		
}

#define MAXV     1.5 // velocity in m/S
#define MAXPHI 20.0 // front steering angle in deg
#define MAX_ACC_V 6 // m/s^2
#define MAX_VEL_PHI 4 // rad/s

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

  double t0 = vpTime::measureTimeMs();		
  double t1 = vpTime::measureTimeMs();
  double tprev;
  bool ctrc = false;
  double timestamp;
  

  while (!end) {
    tprev = t1;
    t1 = vpTime::measureTimeMs();
    // Measures the velocity and the front steering angle from odometry
    cycab->getOdometry(vm, phim, timestamp); // measured values from odometry
      
    printf("State: t=%.1f s  v=%f m/s and phi=%f deg\n\t", 
	   (timestamp-t0)/1000, vm, vpMath::deg(phim));

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
      // we stop the Cycab by decreasing the velocity and the steering
      // angle to zero
      std::cout << "Cycab stop requested" << std::endl;
      // Velocity decrease to zero
      double sign_v = 0;
      if (vm != 0.) sign_v = fabs(vm)/vm;
      v = vm - MAX_ACC_V*(t1-tprev)/1000*sign_v;
      if (fabs(v) < 0.1) v = 0;

       // Steering decrease to zero
      double sign_phi = 0;
      if (phim != 0.) sign_phi = fabs(phim)/phim;
      phi = phim - MAX_VEL_PHI*(t1-tprev)/1000*sign_phi;
      if (fabs(phi) < vpMath::rad(5)) phi = 0;
           
//       printf("stop requested: vm %f v %f phim %f phi %f sign_phi %f\n", 
// 	     vm, v, phim, phi, sign_phi);
      //v = 0;
      //phi = 0;
    }

    // Send the command
    printf("Send : v %f m/s and phi %f deg\n", v, vpMath::deg(phi));
    cycab->setCommand(v, phi);
            
    vpTime::wait(10);

    if (end && (!ctrc)) { end = false; ctrc=true;} 
  }
  std::cout << "The end" << std::endl;
  return 0;
}
		
#else // VISP_HAVE_CYCAB		
int main()
{
  cout << "Sorry no acces to the cycab" << endl;
}
#endif //VISP_HAVE_CYCAB

