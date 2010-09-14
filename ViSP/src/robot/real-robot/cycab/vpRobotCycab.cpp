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
 * Interface for the car-like Cycab mobile robot.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/

#include <visp/vpConfig.h>
#include <visp/vpRobotCycab.h>

#ifdef VISP_HAVE_CYCAB

/*!
  Initialize the client to be able to communicate with the Cycab server.
  
  By default, the steering mode is set such as only the front steering
  angle is controlled.

  To change this behavior, use setDualSteering(true) function.
*/
vpRobotCycab::vpRobotCycab()
{
#ifdef VISP_HAVE_CYCABTK_OLD
  // Old low level controller based on Syndex (obsolete)
  end = false;
  cycab = NULL;
  sprintf(servername, "cycab-hf1");
  cycab = new EtherCycab(servername, &LTR);
  if (!cycab->doInitialization(&end, 50, 0, false)) {
    printf("ERROR: Can't connect to %s\n", servername);
    exit(-1);
  }
  cycab->releaseSecurity();
  printf("Control limits : phi in [%f:%f], V in [%f:%f]\n",
         cycab->minAllowedSteering(),
	 cycab->maxAllowedSteering(),
         cycab->minAllowedSpeed(),
	 cycab->maxAllowedSpeed());
#elif defined VISP_HAVE_CYCABTK
  // New low level controller based on Syndex (to use)
  std::string name = "cycab";
  CycabCarCommand cycab_command;
  setDualSteering(false);
  store.connect();
  cycab_commandId = store.lookupVariable( name + cycabCarCommandSuffix );
  cycab_stateId = store.lookupVariable( name + cycabStateSuffix );
  store.writeVariable( cycab_commandId, cycab_command );
#endif
}

/*!
  Close the connection with the Cycab server.
*/
vpRobotCycab::~vpRobotCycab()
{
#ifdef VISP_HAVE_CYCABTK_OLD
  if (cycab != NULL) delete cycab;
  cycab = NULL;
#endif
}

/*!
  Set the steering mode.
  
  \param dual : If false, only the front steering angle is
  conntrolled. If true, front and read steering angles are controlled.

*/
void vpRobotCycab::setDualSteering(bool dual)
{
  dualSteering = dual;
}

/*!
  Send the command to the cycab server.
  
  \param v : Velocity in m/s.
  \param phi : Front steering angle in rad.
 */
void vpRobotCycab::setCommand(double v, double phi)
{
#ifdef VISP_HAVE_CYCABTK_OLD
  // Old low level controller based on Syndex (obsolete)
  cycab->sendCommands(v, phi);
#elif defined VISP_HAVE_CYCABTK
  // New low level controller based on Syndex (to use)
  CycabCarCommand cycab_command;
  store.readVariable( cycab_commandId, cycab_command );
  cycab_command.v = v;
  cycab_command.phi = phi;
  cycab_command.manualDriving = false;
  cycab_command.dualSteering = dualSteering;
  store.writeVariable( cycab_commandId, cycab_command );
#endif
}

/*!
  Get measures from odometry.
  
  \param vmean : Measured mean velocity in m/s. This value is computed by considering 
  the mean velocity between rear-left and rear-right wheel velocities.

  \param phi : Measured front steering angle in rad.

*/
void vpRobotCycab::getOdometry(double &vmean, double &phi)
{
  double timestamp;
  getOdometry(vmean, phi, timestamp);
}

/*!
  Get measures from odometry.
 
  \param v : Measured mean velocity in m/s. This value is computed by considering 
  the mean velocity between rear-left and rear-right wheel velocities.

  \param phi : Measured front steering angle in rad.

  \param timestamp : Time stamp in milli second associates to the measured
  values of \e vmean and \e phi.
*/
void vpRobotCycab::getOdometry(double &vmean, double &phi, double &timestamp)
{
  vmean = 0.;
  phi=0.;
  timestamp=0.;
  
#ifdef VISP_HAVE_CYCABTK_OLD
  // Old low level controller based on Syndex (obsolete)
  Record rec;
  double dsl,dsr,lphi,ltime;
  cycab->waitUpdate();
  cycab->getState(&rec, &dsl, &dsr, &lphi, &ltime);
  //   printf("Ktate %f : phi %f vl %f vr %f\n\t",ltime,
  //          lphi,rec.vmsec[REAR][LEFT],rec.vmsec[REAR][RIGHT]);
  // calculate speed
  vmean = (rec.vmsec[REAR][LEFT] + rec.vmsec[REAR][RIGHT])*0.5;
  phi = lphi;
  timestamp = ltime;
#elif defined VISP_HAVE_CYCABTK
  // New low level controller based on Syndex (to use)
  CycabState cycab_state;
  store.readVariable(cycab_stateId, cycab_state);
  vmean = (cycab_state.v_rear_left + cycab_state.v_rear_right)*0.5; 
  //printf("vl %f vr %f\n", cycab_state.v_rear_left, cycab_state.v_rear_right);
  phi = cycab_state.phi_front;
  timeval tp = store.getTimestamp(cycab_stateId);
  timestamp = 1000.0*tp.tv_sec + tp.tv_usec/1000.0;
#endif
}
/*!
  Get measures from odometry.
  
  \param vfl : Measured front-left wheel velocity in m/s. 
  \param vfr : Measured front-right wheel velocity in m/s. 
  \param vrl : Measured rear-left wheel velocity in m/s. 
  \param vrr : Measured rear-right wheel velocity in m/s. 

  \param phi : Measured front steering angle in rad.

*/
void vpRobotCycab::getOdometry(double &vfl, double &vfr, 
			       double &vrl, double &vrr,
			       double &phi)
{
  double timestamp;
  getOdometry(vfl, vfr, vrl, vrr, phi, timestamp);
}

/*!
  Get measures from odometry.

  \param vfl : Measured front-left wheel velocity in m/s. 
  \param vfr : Measured front-right wheel velocity in m/s. 
  \param vrl : Measured rear-left wheel velocity in m/s. 
  \param vrr : Measured rear-right wheel velocity in m/s. 

  \param phi : Measured front steering angle in rad.

  \param timestamp : Time stamp in milli second associates to the measured
  values of \e vmean and \e phi.
*/
void vpRobotCycab::getOdometry(double &vfl, double &vfr, 
			       double &vrl, double &vrr,
			       double &phi, double &timestamp)
{
  vfr = vfr = vrl = vrr = 0.;
  phi=0.;
  timestamp=0.;
  
#ifdef VISP_HAVE_CYCABTK_OLD
  // Old low level controller based on Syndex (obsolete)
  Record rec;
  double dsl,dsr,lphi,ltime;
  cycab->waitUpdate();
  cycab->getState(&rec, &dsl, &dsr, &lphi, &ltime);
  //   printf("Ktate %f : phi %f vl %f vr %f\n\t",ltime,
  //          lphi,rec.vmsec[REAR][LEFT],rec.vmsec[REAR][RIGHT]);
  // calculate speed
  vfl = rec.vmsec[FRONT][LEFT];
  vfr = rec.vmsec[FRONT][RIGHT];
  vrl = rec.vmsec[REAR][LEFT];
  vrr = rec.vmsec[REAR][RIGHT];
  phi = lphi;
  timestamp = ltime;
#elif defined VISP_HAVE_CYCABTK
  // New low level controller based on Syndex (to use)
  CycabState cycab_state;
  store.readVariable(cycab_stateId, cycab_state);
  vfl = cycab_state.v_front_left;
  vfr = cycab_state.v_front_right;
  vrl = cycab_state.v_rear_left;
  vrr = cycab_state.v_rear_right;

  phi = cycab_state.phi_front;
  timeval tp = store.getTimestamp(cycab_stateId);
  timestamp = 1000.0*tp.tv_sec + tp.tv_usec/1000.0;
#endif
}

/*!
  Get the joystick positions.
  
  \param x : Joystick left/right position. 
  \param y : Joystick front/rear position. 
 */
void vpRobotCycab::getJoystickPosition(double &x, double &y)
{
  double timestamp;
  getJoystickPosition(x, y, timestamp);
}

/*!
  Get the joystick positions.
  
  \param x : Joystick left/right position. 
  \param y : Joystick front/rear position. 
  \param timestamp : Time stamp in milli second associates to the measured
  values of \e x and \e y.
 */
void vpRobotCycab::getJoystickPosition(double &x, double &y, double &timestamp)
{
#ifdef VISP_HAVE_CYCABTK_OLD
  // Old low level controller based on Syndex (obsolete)
  x = 0.;
  y = 0.;
  timestamp = 0.;
#elif defined VISP_HAVE_CYCABTK
  // New low level controller based on Syndex (to use)
  CycabState cycab_state;
  store.readVariable(cycab_stateId, cycab_state);
  x = cycab_state.joy_x; 
  y = cycab_state.joy_y; 
  timeval tp = store.getTimestamp(cycab_stateId);
  timestamp = 1000.0*tp.tv_sec + tp.tv_usec/1000.0;
#endif
}


#endif
