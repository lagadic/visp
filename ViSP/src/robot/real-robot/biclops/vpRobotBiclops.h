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
 * Interface for the Biclops robot.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/

#include <visp/vpConfig.h>

#ifdef VISP_HAVE_BICLOPS

#ifndef __vpROBOT_BICLOPS_H
#define __vpROBOT_BICLOPS_H


/* ------------------------------------------------------------------------ */
/* --- INCLUDES ----------------------------------------------------------- */
/* ------------------------------------------------------------------------ */

/* --- GENERAL --- */
#include <iostream>
#include <stdio.h>
#include <pthread.h>


/* --- ViSP --- */
#include <visp/vpRobot.h>
#include <visp/vpBiclops.h>
#include <visp/vpColVector.h>
#include <visp/vpDebug.h>
#include <visp/vpVelocityTwistMatrix.h>
#include <visp/vpRobotBiclopsController.h>


/* ------------------------------------------------------------------------ */
/* --- CLASS -------------------------------------------------------------- */
/* ------------------------------------------------------------------------ */



/*!


  \class vpRobotBiclops

  \ingroup Biclops RobotDriver

  \brief Interface for the biclops, pan, tilt head control.

  See http://www.traclabs.com/biclopspt.html for more details.

  This class provide a position and a speed control interface for the biclops
  head. To manage the biclops joint limits in speed control, a control loop is
  running in a seperate thread (see vpRobotBiclopsSpeedControlLoop()).

  The control of the head is done by vpRobotBiclopsController class.

  \warning Velocity control mode is not exported from the top-level Biclops API
  class provided by Traclabs. That means that there is no protection in this
  mode to prevent an axis from striking its hard limit. In position mode,
  Traclabs put soft limits in that keep any command from driving to a position
  too close to the hard limits. In velocity mode this protection does not exist
  in the current API.

  \warning With the understanding that hitting the hard limits at full
  speed/power can damage the unit, damage due to velocity mode commanding is
  under user responsibility.


*/
class VISP_EXPORT vpRobotBiclops
  :
  public vpBiclops,
  public vpRobot
{

private:

  /*! \brief No copy contructor allowed.   */
  vpRobotBiclops (const vpRobotBiclops & ass);

private:
  static bool robotAlreadyCreated;
  pthread_t control_thread;

  char configfile[FILENAME_MAX]; // Biclops config file

  vpRobotBiclopsController controller;

  double positioningVelocity;

public:

  static const double       defaultPositioningVelocity;


  vpRobotBiclops ();
  vpRobotBiclops (const char * filename);
  virtual ~vpRobotBiclops (void);

  void setConfigFile (const char * filename="/usr/share/BiclopsDefault.cfg");
  void init ();

  /* --- STATE ---------------------------------------------------------- */

  vpRobot::vpRobotStateType   setRobotState (const vpRobot::vpRobotStateType newState);

  /* --- POSITION --------------------------------------------------- */
  void setPosition(const vpRobot::vpControlFrameType frame,
		   const vpColVector &q) ;
  void setPosition (const vpRobot::vpControlFrameType frame,
		    const double &q1, const double &q2) ;
  void setPosition(const char *filename) ;
  void getPosition (const vpRobot::vpControlFrameType frame,
		    vpColVector &q);


  void   setPositioningVelocity (const double velocity);
  double getPositioningVelocity (void);


  /* --- SPEED ---------------------------------------------------------- */

  void setVelocity (const vpRobot::vpControlFrameType frame,
		    const vpColVector & q_dot);


  void getVelocity (const vpRobot::vpControlFrameType frame,
		    vpColVector & q_dot);

  vpColVector getVelocity (const vpRobot::vpControlFrameType frame);

public:
  void get_cMe(vpHomogeneousMatrix &_cMe) ;
  void get_cVe(vpVelocityTwistMatrix &_cVe) ;
  void get_eJe(vpMatrix &_eJe)  ;
  void get_fJe(vpMatrix &_fJe)  ;

  void stopMotion() ;

  void getCameraDisplacement(vpColVector &d);
  void getArticularDisplacement(vpColVector &d);
  void getDisplacement(const vpRobot::vpControlFrameType frame, vpColVector &d);

  bool readPositionFile(const char *filename, vpColVector &q)  ;

  static void * vpRobotBiclopsSpeedControlLoop (void * arg);

private:
  vpColVector q_previous;
  bool controlThreadCreated;
};



#endif /* #ifndef __vpROBOT_BICLOPS_H */


/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */

#endif
