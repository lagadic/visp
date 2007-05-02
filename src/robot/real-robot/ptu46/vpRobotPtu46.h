/****************************************************************************
 *
 * $Id: vpRobotPtu46.h,v 1.5 2007-05-02 13:29:41 fspindle Exp $
 *
 * Copyright (C) 1998-2006 Inria. All rights reserved.
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
 * Interface for the ptu-46 robot.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/


#include <visp/vpConfig.h>
#ifdef VISP_HAVE_PTU46


#ifndef __vpROBOT_PTU46_H
#define __vpROBOT_PTU46_H


/* ------------------------------------------------------------------------ */
/* --- INCLUDES ----------------------------------------------------------- */
/* ------------------------------------------------------------------------ */

/* --- GENERAL --- */
#include <iostream>
#include <stdio.h>


/* --- ViSP --- */
#include <visp/vpRobot.h>
#include <visp/vpPtu46.h>
#include <visp/vpColVector.h>
#include <visp/vpDebug.h>
#include <visp/vpTwistMatrix.h>

#include <ptu.h> // Contrib for Ptu-46 robot

/* ------------------------------------------------------------------------ */
/* --- CLASS ------------------------------------------------------------- */
/* ------------------------------------------------------------------------ */


/*!

  \brief Interface for the Directed Perception ptu-46 pan, tilt head .

  See http://www.DPerception.com for more details.

  This class provide a position and a speed control interface for the ptu-46
  head.

*/
class VISP_EXPORT vpRobotPtu46
  :
  public vpPtu46,
  public vpRobot
{

private:

  /*! \brief No copy contructor allowed.   */
  vpRobotPtu46 (const vpRobotPtu46 & ass);

  /*! Object to control. This is a contribution. */
  Ptu ptu;

private:
  static bool robotAlreadyCreated;
  double      positioningVelocity;
  int         velocityMesureTempo;
  char	      *device;

public:

  static const double       defaultPositioningVelocity;

  vpRobotPtu46 (const char *device="/dev/ttyS0");
  vpRobotPtu46 (vpRobotPtu46 * pub);
  virtual ~vpRobotPtu46 (void);

  void init ();


  /* --- STATE ------------------------------------------------------------- */

  vpRobot::RobotStateType   setRobotState (vpRobot::RobotStateType newState);

  /* --- POSITION --------------------------------------------------- */

  void setPosition(const vpRobot::ControlFrameType frame,
		   const vpColVector &q) ;
  void setPosition (const vpRobot::ControlFrameType frame,
		    const double &q1, const double &q2) ;
  void setPosition(const char *filename) ;
  void getPosition (const vpRobot::ControlFrameType frame,
		    vpColVector &q);


  void   setPositioningVelocity (const double velocity);
  double getPositioningVelocity (void);


  /* --- SPEED ---------------------------------------------------------- */

  void setVelocity (const vpRobot::ControlFrameType frame,
		    const vpColVector & q_dot);


  void getVelocity (const vpRobot::ControlFrameType frame,
		    vpColVector & q_dot);

  vpColVector getVelocity (const vpRobot::ControlFrameType frame);

public:
  void get_cMe(vpHomogeneousMatrix &_cMe) ;
  void get_cVe(vpTwistMatrix &_cVe) ;
  void get_eJe(vpMatrix &_eJe)  ;
  void get_fJe(vpMatrix &_fJe)  ;

  void stopMotion() ;

  void getCameraDisplacement(vpColVector &d);
  void getArticularDisplacement(vpColVector  &d);
  void getDisplacement(vpRobot::ControlFrameType  frame, vpColVector &q);

  bool readPositionFile(const char *filename, vpColVector &q);
};



#endif /* #ifndef __vpROBOT_PTU46_H */


/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */

#endif
