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
 * Interface for the car-like Cycab mobile robot.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/

#include <visp/vpConfig.h>
#include <visp/vpRobotCycab.h>

#if defined(VISP_HAVE_CYCABTK) || defined(VISP_HAVE_CYCABTK_OLD)

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
  Get the actual command applied to the vehicle from the odometry.
  
  \param v : Velocity in m/s.

  \param phi : Front steering angle in rad.

*/
void vpRobotCycab::getCommand(double &v, double &phi)
{
  double timestamp;
  getCommand(v, phi, timestamp);
}

/*!
  Get the actual command applied to the vehicle from the odometry.
  
  \param v : Velocity in m/s.

  \param phi : Front steering angle in rad.

  \param timestamp : Time stamp in second associates to the measured
  values of \e v and \e phi.
*/
void vpRobotCycab::getCommand(double &v, double &phi, double &timestamp)
{
  v = 0.;
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
  v = (rec.vmsec[REAR][LEFT] + rec.vmsec[REAR][RIGHT])*0.5;
  phi = lphi;
  timestamp = ltime;
#elif defined VISP_HAVE_CYCABTK
  // New low level controller based on Syndex (to use)
  CycabState cycab_state;
  store.readVariable(cycab_stateId, cycab_state);
  v = (cycab_state.v_rear_left + cycab_state.v_rear_left)*0.5; 
  //printf("vl %f vr %f\n", cycab_state.v_rear_left, cycab_state.v_rear_left);
  phi = cycab_state.phi_front;
  timestamp = cycab_state.time_stamp;
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
  \param timestamp : Time stamp in second associates to the measured
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
  timestamp = cycab_state.time_stamp;
#endif
}


#endif
