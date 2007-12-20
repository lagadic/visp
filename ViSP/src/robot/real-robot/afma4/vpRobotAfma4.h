/****************************************************************************
 *
 * $Id: vpRobotAfma4.h,v 1.7 2007-12-20 08:15:28 fspindle Exp $
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
 * Interface for the Irisa's Afma4 robot.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/

#include <visp/vpConfig.h>

#ifdef VISP_HAVE_AFMA4

#ifndef __vpROBOT_AFMA4_H
#define __vpROBOT_AFMA4_H


/* ------------------------------------------------------------------------ */
/* --- INCLUDES ----------------------------------------------------------- */
/* ------------------------------------------------------------------------ */

/* --- ViSP --- */

#include <visp/vpRobot.h>
#include <visp/vpAfma4.h>
#include <visp/vpColVector.h>
#include <visp/vpDebug.h>

/* --- GENERAL --- */
#include <iostream>
#include <stdio.h>

#include <visp/vpRobotAfma4Contrib.h>
#include <visp/vpTwistMatrix.h>
/* ------------------------------------------------------------------------ */
/* --- CLASSE ------------------------------------------------------------- */
/* ------------------------------------------------------------------------ */


/**
 * \brief Implementation de la classe vpRobot permettant de diriger le
 * robot AFMA4.
 *
 * La classe vpRobotAfma4 reprend toutes les specs de la classe CRobot et
 * les implementent toutes.
 */
class vpRobotAfma4
  :
  public vpAfma4,
  public vpRobot
{

private: /* Methodes implicites interdites. */

  /** \brief Constructeur par clonage interdit.
   */
  vpRobotAfma4 (const vpRobotAfma4 & ass);

private: /* Attributs prives. */

  /** \brief Vrai ssi aucun objet de la classe vpRobotAfma4 n'existe.
   *
   * Il ne peut exister simultanement qu'un seul objet de la classe
   * vpRobotAfma4, car il correspond a un seul robot AFMA4. Creer
   * simultanement deux objets peut engendrer des conflits. Le constructeur
   * lance une erreur si le champ n'est pas FAUX puis positionne le champ
   * a VRAI. Seul le destructeur repositionne le champ a FAUX, ce qui
   * alors la creation d'un nouvel objet.
   */
  static bool               robotAlreadyCreated;

public:  /* Attributs publiques. */

  /* Vitesse maximale du robot lors d'un positionnement. */
  double                    positioningVelocity;
  /** Attente pour la lecture de la vitesse courrante. */
  int                       velocityMesureTempo;

  /* On maintient construit un objet de chaque classe de communication. */

  /** Objet de communication avec le robot en position (set et get). */
  st_position_Afma4         communicationPosition;
  st_mouvement_Afma4        communicationVelocity;

public:  /* Construction Destruction. */

  /** Constructeur avec un reference sur la partie publique.
   * \throw ERRUniciteRobot si un objet vpRobotAfma4 a deja ete cree.
   */
  vpRobotAfma4 (vpRobotAfma4 * pub);


public:  /* Attributs */



public: /* Methodes */

  /* Initialise les connexions avec la carte VME et demare le robot.
   * La fonction est lancee lors de la construction. Il s'agit uniquement
   * d'une factorisation du code.
   * \throw
   *   - ERRConstruction si une erreur survient lors de l'ouverture du VME.
   *   - ERRCommunication si une erreur survient lors de l'initialisation de
   * l'afma4.
   */
  void init (void);

public:  /* Constantes */

  /** Vitesse maximale par default lors du positionnement du robot.
   * C'est la valeur a la construction de l'attribut prive \a
   * positioningVelocity. Cette valeur peut etre changee par la fonction
   * #setPositioningVelocity.
   */
  static const double       defaultPositioningVelocity; // = 20.0;

  /** Nombre d'articulation, i.e. taille des vecteurs articulaires. */
  static const int          nbArticulations; // = 4;

public:  /* Attributs publiques */

public:  /* Methode publiques */

  /** \brief Constructeur vide, le seul possible.
   *
   * \throw ERRUniciteRobot si un objet vpRobotAfma4 a deja ete cree. Un
   * seul vpRobotAfma4 peut exister a un temps donner. Si cette erreur est
   * lancee, cela veut dire que le precedent vpRobotAfma4 cree n'a pas
   * ete correctement detruit.
   */
  vpRobotAfma4 (void);

  /** \brief Destructeur.
   *
   * \throw ERRCommunication si une erreur survient lors de la fermeture
   * des communications avec le robot.
   * \throw ERRMauvaisEtatRobot si le robot n'etait pas arrete au moment de
   * la destruction de l'objet. Dans ce cas le robot est arrete et les
   * communications sont fermee, avant le lancement de l'erreur. */
  virtual ~vpRobotAfma4 (void);


  /* --- ETAT ------------------------------------------------------------- */

  vpRobot::vpRobotStateType   setRobotState (vpRobot::vpRobotStateType newState);

  /* --- POSITIONNEMENT --------------------------------------------------- */

  //! set a displacement (frame as to ve specified)
  void setPosition(const vpRobot::vpControlFrameType frame,
		   const vpColVector &q) ;
  void setPosition (const vpRobot::vpControlFrameType repere,
		    const double q1, const double q2,
		    const double q3, const double q4) ;
  void getPosition (const vpRobot::vpControlFrameType repere,
		    vpColVector &r);


  void                      setPositioningVelocity (const double velocity);
  double                    getPositioningVelocity (void);

  void setZoom(int zoom) ;
  int  getZoom() ;
  void setIris(int dia) ;
  int  getIris() ;
  void setAutoIris(bool on);
  void setFocus(int focus) ;
  int  getFocus() ;



  /* --- VITESSE ---------------------------------------------------------- */

  void setVelocity (const vpRobot::vpControlFrameType repere,
		    const vpColVector & r_dot);


  void getVelocity (const vpRobot::vpControlFrameType repere,
		    vpColVector & r_dot);

  vpColVector getVelocity (const vpRobot::vpControlFrameType repere);

public:
  void get_cMe(vpHomogeneousMatrix &_cMe) ;
  void get_cVe(vpTwistMatrix &_cVe) ;
  //! get the robot Jacobian expressed in the end-effector frame
  void get_eJe(vpMatrix &_eJe)  ;
  //! get the robot Jacobian expressed in the robot reference frame
  void get_fJe(vpMatrix &_fJe)  ;

  void stopMotion() ;

  /* --- Vecteur vers double --- */
  static inline void          VD4_mmrad_mrad(vpColVector& r);
  /** Vecteur de dimension 6: de m et dg vers m et rad. */
  static inline void
  VD4_mdg_mrad (const vpColVector &input, double * output);

  /** Vecteur de dimension 4: de m et dg vers m et rad. */
  static inline void
  VD4_mrad_mmrad (const vpColVector & input, double * output);

  /* --- Double vers vecteur --- */

  /** Vecteur de dimension 4: de mm et dg vers mm et dg. */
  static inline void
  VD4_mmdg_mdg (const double * input, vpColVector & output);

  /** Vecteur de dimension 4: de mm et rad vers mm et rad. */
  static inline void
  VD4_mmrad_mrad (const double * input, vpColVector & output);

  // get a displacement expressed in the camera frame
  void getCameraDisplacement(vpColVector &v);
  // get a displacement expressed  in the articular frame
  void getArticularDisplacement(vpColVector  &qdot);

  // get a displacement (frame as to ve specified)
  void getDisplacement(vpRobot::vpControlFrameType  frame, vpColVector &q);

  void move(char *name) ;
  int  readPosFile(char *name, vpColVector &v)  ;
};



#endif /* #ifndef __vpROBOT_AFMA4_H */


/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */

#endif
