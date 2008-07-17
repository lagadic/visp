/****************************************************************************
 *
 * $Id: vpRobotAfma6.h,v 1.17 2008-07-17 20:14:58 fspindle Exp $
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
 * Interface for the Irisa's Afma6 robot controlled by an Adept MotionBlox.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/

#ifndef vpRobotAfma6_h
#define vpRobotAfma6_h

#include <visp/vpConfig.h>

#ifdef VISP_HAVE_AFMA6

#include <iostream>
#include <stdio.h>



/* ------------------------------------------------------------------------ */
/* --- INCLUDES ----------------------------------------------------------- */
/* ------------------------------------------------------------------------ */

/* --- ViSP --- */

#include <visp/vpRobot.h>
#include <visp/vpColVector.h>
#include <visp/vpDebug.h>
#include <visp/vpAfma6.h>

// low level controller api
extern "C" {
#  include "irisa.h"
#  include "trycatch.h"
}

/* ------------------------------------------------------------------------ */
/* --- CLASSE ------------------------------------------------------------- */
/* ------------------------------------------------------------------------ */


/*!

  Implementation of the vpRobot class in order to control Irisa's Afma6 robot.

*/
class VISP_EXPORT vpRobotAfma6
  :
  public vpAfma6,
  public vpRobot
{

private: /* Methodes implicites interdites. */

  /*!
    Copy contructor not allowed.
   */
  vpRobotAfma6 (const vpRobotAfma6 & ass);

private: /* Attributs prives. */

  /** \brief Vrai ssi aucun objet de la classe vpRobotAfma6 n'existe.
   *
   * Il ne peut exister simultanement qu'un seul objet de la classe
   * vpRobotAfma6, car il correspond a un seul robot AFMA6. Creer
   * simultanement deux objets peut engendrer des conflits. Le constructeur
   * lance une erreur si le champ n'est pas FAUX puis positionne le champ
   * a VRAI. Seul le destructeur repositionne le champ a FAUX, ce qui
   * alors la creation d'un nouvel objet.
   */
  static bool robotAlreadyCreated;

  double positioningVelocity;

  // Variables used to compute the measured velocities (see getVelocity() )
  vpColVector q_prev_getvel;
  vpHomogeneousMatrix fMc_prev_getvel;
  double time_prev_getvel;
  bool first_time_getvel;

public: /* Methodes */

  void init (void);
  void init (vpAfma6::vpAfma6CameraRobotType camera,
             vpCameraParameters::vpCameraParametersProjType
	     projModel = vpCameraParameters::perspectiveProjWithoutDistortion);

public:  /* Constantes */

  /** Vitesse maximale par default lors du positionnement du robot.
   * C'est la valeur a la construction de l'attribut prive \a
   * positioningVelocity. Cette valeur peut etre changee par la fonction
   * #setPositioningVelocity.
   */
  static const double defaultPositioningVelocity; // = 20.0;

public:  /* Methode publiques */

  vpRobotAfma6 (void);
  virtual ~vpRobotAfma6 (void);


  /* --- ETAT ------------------------------------------------------------- */

  vpRobot::vpRobotStateType setRobotState (vpRobot::vpRobotStateType newState);

  /* --- POSITIONNEMENT --------------------------------------------------- */
  void setPosition(const vpRobot::vpControlFrameType frame,
		   const vpColVector &position) ;
  void setPosition (const vpRobot::vpControlFrameType frame,
		    const double pos1, const double pos2, const double pos3,
		    const double pos4, const double pos5, const double pos6) ;
  void getPosition (const vpRobot::vpControlFrameType frame,
		    vpColVector &position);


  void   setPositioningVelocity (const double velocity);
  double getPositioningVelocity (void);

  /* --- VITESSE ---------------------------------------------------------- */

  void setVelocity (const vpRobot::vpControlFrameType frame,
		    const vpColVector & velocity);


  void getVelocity (const vpRobot::vpControlFrameType frame,
		    vpColVector & velocity);

  vpColVector getVelocity (const vpRobot::vpControlFrameType frame);

public:
  void get_cMe(vpHomogeneousMatrix &_cMe) ;
  void get_cVe(vpTwistMatrix &_cVe) ;
  //! get the robot Jacobian expressed in the end-effector frame
  void get_eJe(vpMatrix &_eJe)  ;
  //! get the robot Jacobian expressed in the robot reference frame
  void get_fJe(vpMatrix &_fJe)  ;

  void stopMotion() ;
  void powerOff() ;

  void move(const char *filename) ;
  static bool readPosFile(const char *filename, vpColVector &q)  ;
  static bool savePosFile(const char *filename, const vpColVector &q)  ;

  void openGripper() ;
  void closeGripper() ;

  void getCameraDisplacement(vpColVector &d);
  void getArticularDisplacement(vpColVector  &d);
  void getDisplacement(vpRobot::vpControlFrameType  frame, vpColVector &d);
};





/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */

#endif
#endif /* #ifndef vpRobotAfma6_h */
