/****************************************************************************
 *
 * $Id$
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
 * Interface for the Biclops robot.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/

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

#include <visp/vpConfig.h>

#include "Biclops.h"	// Contrib for Biclops robot
#include "PMDUtils.h"  	// Contrib for Biclops robot

#ifdef WIN32
class VISP_EXPORT Biclops; //needed for dll creation
#endif

/* ------------------------------------------------------------------------ */
/* --- CLASS ------------------------------------------------------------- */
/* ------------------------------------------------------------------------ */

/*!

  \class vpRobotBiclopsController

  \ingroup Biclops

  \brief Interface to Biclops, pan, tilt, verge head for computer vision
  applications.

  See http://www.traclabs.com/tracbiclops.htm for more details.

  This class uses libraries libBiclops.so, libUtils.so and libPMD.so and
  includes Biclops.h and PMDUtils.h provided by Traclabs.

*/
class VISP_EXPORT vpRobotBiclopsController
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
  virtual ~vpRobotBiclopsController();
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
