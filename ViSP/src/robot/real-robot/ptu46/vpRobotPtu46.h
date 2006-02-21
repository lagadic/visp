/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet Lagadic / IRISA, 2006
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      vpRobotPtu46.h
 * Project:   ViSP robot
 * Author:    Fabien Spindler
 *
 * Version control
 * ===============
 *
 *  $Id: vpRobotPtu46.h,v 1.2 2006-02-21 11:14:43 fspindle Exp $
 *
 * Description
 * ============
 *
 *
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

#include <visp/vpConfig.h>
#ifdef HAVE_ROBOT_PTUEVI


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
class vpRobotPtu46
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
  ~vpRobotPtu46 (void);

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
