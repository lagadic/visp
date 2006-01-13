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
 *  $Id: vpRobotPtu46.h,v 1.1 2006-01-13 18:15:21 fspindle Exp $
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
#include <iostream>                /* Classe std::ostream.              */
#include <stdio.h>                /* Classe std::ostream.              */


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


/**
 * \brief Implementation de la classe vpRobot permettant de diriger le
 * robot AFMA4.
 *
 * La classe vpRobotAfma4 reprend toutes les specs de la classe CRobot et
 * les implementent toutes.
 */
class vpRobotPtu46
  :
  public vpPtu46,
  public vpRobot
{

private: /* Methodes implicites interdites. */

  /** \brief Constructeur par clonage interdit.
   */
  vpRobotPtu46 (const vpRobotPtu46 & ass);

  /*! Object to control. This is a contribution. */
  Ptu ptu;

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

public:  /* Construction Destruction. */

  /** Constructeur avec un reference sur la partie publique.
   * \throw ERRUniciteRobot si un objet vpRobotPtu46 a deja ete cree.
   */
  vpRobotPtu46 (vpRobotPtu46 * pub);


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
  static const double       defaultPositioningVelocity;

  /** Nombre d'articulation, i.e. taille des vecteurs articulaires. */
  static const int          nbArticulations;

public:  /* Attributs publiques */

public:  /* Methode publiques */

  /** \brief Constructeur vide, le seul possible.
   *
   * \throw ERRUniciteRobot si un objet vpRobotAfma4 a deja ete cree. Un
   * seul vpRobotAfma4 peut exister a un temps donner. Si cette erreur est
   * lancee, cela veut dire que le precedent vpRobotAfma4 cree n'a pas
   * ete correctement detruit.
   */
  vpRobotPtu46 (void);

  /** \brief Destructeur.
   *
   * \throw ERRCommunication si une erreur survient lors de la fermeture
   * des communications avec le robot.
   * \throw ERRMauvaisEtatRobot si le robot n'etait pas arrete au moment de
   * la destruction de l'objet. Dans ce cas le robot est arrete et les
   * communications sont fermee, avant le lancement de l'erreur. */
  ~vpRobotPtu46 (void);


  /* --- SATE ------------------------------------------------------------- */

  vpRobot::RobotStateType   setRobotState (vpRobot::RobotStateType newState);

  /* --- POSITION --------------------------------------------------- */

  //! set a displacement (frame as to ve specified)
  void setPosition(const vpRobot::ControlFrameType frame,
		   const vpColVector &q) ;
  void setPosition (const vpRobot::ControlFrameType repere,
		    const double q1, const double q2) ;
  void getPosition (const vpRobot::ControlFrameType repere,
		    vpColVector &r);


  void   setPositioningVelocity (const double velocity);
  double getPositioningVelocity (void);


  /* --- SPEED ---------------------------------------------------------- */

  void setVelocity (const vpRobot::ControlFrameType repere,
		    const vpColVector & r_dot);


  void getVelocity (const vpRobot::ControlFrameType repere,
		    vpColVector & r_dot);

  vpColVector getVelocity (const vpRobot::ControlFrameType repere);

public:
  void get_cMe(vpHomogeneousMatrix &_cMe) ;
  void get_cVe(vpTwistMatrix &_cVe) ;
  //! get the robot Jacobian expressed in the end-effector frame
  void get_eJe(vpMatrix &_eJe)  ;
  //! get the robot Jacobian expressed in the robot reference frame
  void get_fJe(vpMatrix &_fJe)  ;

  void stopMotion() ;

  // get a displacement expressed in the camera frame
  void getCameraDisplacement(vpColVector &v);
  // get a displacement expressed  in the articular frame
  void getArticularDisplacement(vpColVector  &qdot);

  // get a displacement (frame as to ve specified)
  void getDisplacement(vpRobot::ControlFrameType  frame, vpColVector &q);

  void move(char *name) ;
  int  readPosFile(char *name, vpColVector &v)  ;
};



#endif /* #ifndef __vpROBOT_PTU46_H */


/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */

#endif
