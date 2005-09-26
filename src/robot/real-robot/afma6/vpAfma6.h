/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet LAGADIC / IRISA, 2004
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      vpAfma6.h
 * Project:   ViSP chapitre robot
 * Author:    Nicolas Mansard
 *
 * Version control
 * ===============
 *
 *
 *
 *  $Id: vpAfma6.h,v 1.3 2005-09-26 15:56:54 fspindle Exp $
 *
 * Description
 * ============
 *
 * Definition de la classe vpAfma6.
 *
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

#include <visp/vpConfig.h>
#ifdef HAVE_ROBOT_AFMA6

#ifndef __vpAFMA6_H
#define __vpAFMA6_H


/* ----------------------------------------------------------------------- */
/* --- INCLUDES -------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/* --- GENERAL --- */
#include <iostream>                /* Definition class ostream.               */

/* --- ViSP --- */
#include <visp/vpMatrix.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpPoseVector.h>

#include <visp/vpMath.h>
#include <visp/vpRxyzVector.h>
#include <visp/vpTranslationVector.h>
#include <visp/vpTwistMatrix.h>


/* ----------------------------------------------------------------------- */
/* --- CLASSE ------------------------------------------------------------ */
/* ----------------------------------------------------------------------- */

/** \brief Lecture du fichiers de config du robot Afma6 et fonctionnalites en
 * decoulant (calcul du jacobien, du MGD, ...).
 *
 */

class vpAfma6
{

public: /* Constantes */

  /** Nombre d'articulations du robot. */
  static const int articulationsNb; // 6

  /** Nom du fichier ou est rangee la liste de parametres chez Fabien. */
  static const char * const PARAMETRES_AFMA6_FILENAME ;

  /** Nom du fichier de parametres MPI de la cam XC77. */
  static const char * const CONST_MPI_XC77 ;

  /** Nom du fichier de parametres MPI de la cam HF. */
  static const char * const CONST_MPI_HF ;

  /** Nom du fichier de parametres MPI de la cam IEEE1394. */
  static const char * const CONST_MPI_IEEE1394 ;

  /** Differentes cameras installees sur l'afma6. */
  enum CameraRobotType
    {
      CAMERA_XC77_12MM,
      CAMERA_HF,
      CAMERA_IEEE1394_12MM,
    } ;
  /** Camera utilisee par default. */
  static const CameraRobotType defaultCameraRobot;// = CAMERA_XC77_12MM;

public: /* Attributs publiques */

  float                       Kp [6];
  float                       Kd [6];
  float                       Ki [6];

  /** Bornes inferieures des intervalles du domaine articulaire,
   * unite 'top' (i.e. un pas du moteur). */
  long                        QMax [6];           /* En mm et rad. */
  /** Bornes inferieures des intervalles du domaine articulaire,
   * unite 'top' (i.e. un pas du moteur). */
  long                        QMin [6];           /* En mm et rad. */
  /** Taille d'un top, l'unite de mesure des intervalles articulaires,
   * unite mm et rad. */
  float                       top [6];

  long                        RstQm [6];
  long                        SensDep [6];
  long                        EpsMax [6];
  long                        TiMax [6];
  long                        AccMax [6];
  long                        VitMax [6];
  long                        ErrTMax [6];

  float                       l;
  float                       coupl;

  /** Vecteur de representation minimale (Theta.U) de la rotation entre
   * les reperes poignet (R6) et image (R7). C'est la partie rotation
   * de la matrice RPI (unite dg). */
  vpRxyzVector                rrpi;
  /** Vecteur de translation entre les reperes poignet (R6) et image (R7).
   * C'est la partie translation de la matrice RPI (unite mm).*/
  vpTranslationVector         trpi;

  /** Bornes superieures des intervalles du domaine articulaire,
   * unite mm et rad. */
  double                      jointMax [6];       /* En m  et rad. */
  /** Bornes inferieures des intervalles du domaine articulaire,
   * unite mm et rad. */
  double                      jointMin [6];       /* En m  et rad. */
  /** Centre des intervalles du domaine articulaire, unite mm et rad. */
  double                      jointMiddle [6];    /* En m  et rad. */

  /** Matrice homogene de passage entre le poignet (axe 6) et l'image
   * (~ axe 7), i.e. matrice r67. */
  vpHomogeneousMatrix          rpi;                /* En m  et rad. */

  bool                        FlagMod;
  bool                        FlagReset;


  /** Vrai ssi la classe a deja ete initiliasee une fois. */
  bool                        dejaInitialisee;

public: /* Methodes publiques */

  /** \brief Constructeur vide. */
  vpAfma6 (void);

  /** \brief Lecture des fichiers de configuration.   */
  void                        init (const char * filename,
				    const char * filenameMPI);

  /* \brief Lecture des fichiers de configuration.   */
  void                        init (vpAfma6::CameraRobotType camera);

  /** \brief Initialisation a l'aide du fichier par default.    */
  void                        init (void);

  /** \brief Affichage.
   *
   * Affichage de tous les champs de la classe en mode texte.
   */
  friend ostream & operator << (ostream & os,
				const vpAfma6 & constant);

private: /* Methodes privees. */

  /** Initialise les tableaux de valeurs limites du domaine articulaire
   * apres la lecture du fichier de config.
   * Les donnees sont lues dans les variables Qmax Qmin et top.
   * Les resultats sont enregistres dans jointMax, jointMin et
   * jointMiddle.
   */
  void                        initJointLimits (void);
  /** Initialise la matrice homogene RPI de passage entre le poignet et
   * l'image, a partir des donnees lues dans le fichier de config.
   */
  void                        initRpi (void);

  /* Lecture d'un fichier de config. */
  void parseConfigFile (const char * filename);
public:

  /** \brief Calcul le jacobien inverse.    */

  void  computeInverseJacobian (const vpColVector & q, vpMatrix & Jinverse);

  /** \brief Calcul le jacobien inverse.    */

  vpMatrix  computeInverseJacobian (const vpColVector & q);

public: /* Constantes publiques */

  /** \brief parametre par default des fonctions getActivationThreshold.  */
  static const double rhoDefault;

protected: /* Attributs prives */

  /** Seuil d'activation de l'evitement de butees (en pourcentage).   */
  double          rho;

public: /* Methodes publiques */

  /* --- VALEURS ---------------------------------------------------------- */

  /** @name Valeurs des butees */
  //@{

  /** \brief Renvoie les butees basses du robot.   */
  void getJointLimitsMin (double jointMin[6]) const;
  //void getJointLimitsMin (double jointMin[articulationsNb]) const;

  /** \brief Renvoie les butees hautes du robot.    */
  void getJointLimitsMax (double jointMax[6]) const;
  //  void getJointLimitsMax (double jointMax[articulationsNb]) const;

  //@}

  /* --- POSITION --------------------------------------------------------- */

  /** @name Position articulaire */
  //@{

  /** \brief Modifie la position pour rester dans l'espace autorise.   */
  void respectJointLimits (vpColVector &q) const;

  /** Verifie si les limites des butees sont respectées.    */
  bool areJointLimitsrespected (const vpColVector & q) const;

  /** \brief Renvoie la position courrante relative aux butees.    */
  void getPositionInJointInterval (const vpColVector &q,
				   double jointPos[6]) const;
  //void getPositionInJointInterval (const vpColVector &q,
  //                                 double jointPos[articulationsNb]) const;

  //@}

  /* --- EVITEMENT -------------------------------------------------------- */

  /** @name Evitement de butees. */
  //@{

  /** \brief Modifie le seuil d'activation des butees.   */
  void            setActivationThreshold (const double rho);

  /** \brief Recupere le seuil d'activation des butees.    */
  void            getActivationThreshold (double &rho) const;

  /** \brief Recupere le seuil d'activation des butees.    */
  double          getActivationThreshold (void) const;

  /** \brief Calcule le gradient de la fonction de potentiel d'evitement
   * d'obstacle.    */
  bool            getAvoidanceGradiant (const vpColVector & q,
					vpColVector & grad);

  /** \brief Renvoie le nombre d'iteration prevues avant de
   * toucher une butée, si on conserve la meme commande.
     */
  int iterationsAvantButees (const vpColVector &q,
			     const vpColVector &q_dot,
			     const int limiteIteration) const;

  //@}

public:

  //! \brief Calcul le MGD du robot.
  void  computeMGD (const vpColVector &q, vpHomogeneousMatrix & fMc);

  //! \brief Calcul le MGD.
  vpHomogeneousMatrix          computeMGD (const vpColVector & q);
  //! \brief Calcul le MGD du robot sous forme d'un vecteur.
  void  computeMGD (const vpColVector & q,  vpPoseVector & r);

  void get_cMe(vpHomogeneousMatrix &_cMe) ;
  void get_cVe(vpTwistMatrix &_cVe) ;
  //! get the robot Jacobian expressed in the end-effector frame
  void get_eJe(const vpColVector &q, vpMatrix &_eJe)  ;
  //! get the robot Jacobian expressed in the robot reference frame
  void get_fJe(const vpColVector &q, vpMatrix &_fJe)  ;
};



#endif /* #ifndef __AFMA6_H */





/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */

#endif
