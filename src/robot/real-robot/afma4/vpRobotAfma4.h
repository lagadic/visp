/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet Lagadic / IRISA, 2004
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      vpRobotAfma4.h
 * Project:   ViSP chapitre robot
 * Author:    Nicolas Mansard
 *
 * Version control
 * ===============
 *
 *  $Id: vpRobotAfma4.h,v 1.4 2006-04-19 09:01:21 fspindle Exp $
 *
 * Description
 * ============
 *
 *
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

#include <visp/vpConfig.h>

#ifdef VISP_HAVE_AFMA4

#ifndef __vpROBOT_AFMA4_H
#define __vpROBOT_AFMA4_H


/* ------------------------------------------------------------------------ */
/* --- INCLUDES ----------------------------------------------------------- */
/* ------------------------------------------------------------------------ */

/* --- ViSP --- */

#include <visp/vpRobot.h>   /* Classe CRobot dont herite CRobotAfma4.  */
#include <visp/vpAfma4.h>   /* Constantes du robot (classe vpAfma4).    */
#include <visp/vpColVector.h>   /* Constantes du robot (classe vpAfma4).    */
#include <visp/vpDebug.h>   /* Constantes du robot (classe vpAfma4).    */

/* --- GENERAL --- */
#include <iostream>                /* Classe std::ostream.              */
#include <stdio.h>                /* Classe std::ostream.              */

#include <visp/vpRobotAfma4Contrib.h>   /* Constantes du robot (classe vpAfma4).    */
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
  ~vpRobotAfma4 (void);


  /* --- ETAT ------------------------------------------------------------- */

  vpRobot::RobotStateType   setRobotState (vpRobot::RobotStateType newState);

  /* --- POSITIONNEMENT --------------------------------------------------- */

  //! set a displacement (frame as to ve specified)
  void setPosition(const vpRobot::ControlFrameType frame,
		   const vpColVector &q) ;
  void setPosition (const vpRobot::ControlFrameType repere,
		    const double q1, const double q2,
		    const double q3, const double q4) ;
  void getPosition (const vpRobot::ControlFrameType repere,
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
  void getDisplacement(vpRobot::ControlFrameType  frame, vpColVector &q);

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
