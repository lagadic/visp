/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet Lagadic / IRISA, 2006
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      vpRobotBiclops.h
 * Project:   ViSP robot
 * Author:    Fabien Spindler
 *
 * Version control
 * ===============
 *
 *  $Id: vpRobotBiclopsController.h,v 1.2 2006-04-19 09:01:21 fspindle Exp $
 *
 * Description
 * ============
 *
 *
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

#include <visp/vpConfig.h>

#ifdef VISP_HAVE_BICLOPS

#ifndef __vpROBOT_BICLOPS_CONTROLLER_H
#define __vpROBOT_BICLOPS_CONTROLLER_H


/* ------------------------------------------------------------------------ */
/* --- INCLUDES ----------------------------------------------------------- */
/* ------------------------------------------------------------------------ */

/* --- GENERAL --- */
#include <iostream>                /* Classe std::ostream.              */
#include <stdio.h>                /* Classe std::ostream.              */
#include <pthread.h>                /* Classe std::ostream.              */


#include "Biclops.h"	// Contrib for Biclops robot
#include "PMDUtils.h"  	// Contrib for Biclops robot


/* ------------------------------------------------------------------------ */
/* --- CLASS ------------------------------------------------------------- */
/* ------------------------------------------------------------------------ */

/*!

  \brief Interface to Biclops, pan, tilt, verge head for computer vision
  applications.

  See http://www.traclabs.com/tracbiclops.htm for more details.

  This class uses libraries libBiclops.so, libUtils.so and libPMD.so and
  includes Biclops.h and PMDUtils.h provided by Traclabs.

*/
class vpRobotBiclopsController
{
public:
  typedef enum {
    STOP, /*!< Have to stop the robot. */
    SPEED /*!< Can send the desired speed. */
  } vpControllerStatusType;

public:
#ifndef DOXYGEN_SHOULD_SKIP_THIS
  // SHM
  typedef struct /* ControllerShm_struct */ {
    vpControllerStatusType status[2];
    float q_dot[2];    /*!< Desired speed. */
    float actual_q[2]; /*!< Current measured position of each axes. */
    float actual_q_dot[2]; /*!< Current measured velocity of each axes. */
    bool  jointLimit[2]; /*!< Indicates if an axe is in joint limit. */
  } shmType;
#endif /* DOXYGEN_SHOULD_SKIP_THIS */

public:
  vpRobotBiclopsController();
  ~vpRobotBiclopsController();
  void init(const char *configfile);
  void setPosition(const vpColVector & q, const double percentVelocity);
  void setVelocity(const vpColVector & q_dot);
  vpColVector getPosition();
  vpColVector getActualPosition();
  vpColVector getVelocity();
  vpColVector getActualVelocity();
  PMDAxisControl * getPanAxis()   { return panAxis;   };
  PMDAxisControl * getTiltAxis()  { return tiltAxis;  };
  PMDAxisControl * getVergeAxis() { return vergeAxis; };
  void writeShm(shmType &shm);
  shmType readShm();

private:
  Biclops biclops; // THE interface to Biclops.
  int axisMask;

  // Pointers to each axis (populated once controller is initialized).
  PMDAxisControl *panAxis;
  PMDAxisControl *tiltAxis;
  PMDAxisControl *vergeAxis;

  PMDAxisControl::Profile panProfile;
  PMDAxisControl::Profile tiltProfile;
  PMDAxisControl::Profile vergeProfile;

  shmType shm;

};


#endif /* #ifndef __vpROBOT_BICLOPS_CONTROLLER_H */


/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */

#endif
