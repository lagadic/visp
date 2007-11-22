/****************************************************************************
 *
 * $Id: vpAfma6.h,v 1.11 2007-11-22 09:22:23 fspindle Exp $
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
 * Interface for the Irisa's Afma6 robot.
 *
 * Authors:
 * Nicolas Mansard
 * Fabien Spindler
 *
 *****************************************************************************/


#include <visp/vpConfig.h>
#ifdef VISP_HAVE_AFMA6

#ifndef __vpAFMA6_H
#define __vpAFMA6_H


/* ----------------------------------------------------------------------- */
/* --- INCLUDES -------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/* --- GENERAL --- */
#include <iostream>                /* Definition class std::ostream.               */

/* --- ViSP --- */
#include <visp/vpMatrix.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpPoseVector.h>

#include <visp/vpMath.h>
#include <visp/vpRxyzVector.h>
#include <visp/vpTranslationVector.h>
#include <visp/vpTwistMatrix.h>
#include <visp/vpImage.h>
#include <visp/vpRGBa.h>
#include <visp/vpCameraParameters.h>


/* ----------------------------------------------------------------------- */
/* --- CLASSE ------------------------------------------------------------ */
/* ----------------------------------------------------------------------- */

/** \brief Lecture du fichiers de config du robot Afma6 et fonctionnalites en
 * decoulant (calcul du jacobien, du MGD, ...).
 *
 */

class VISP_EXPORT vpAfma6
{

public: /* Constantes */

  /** Nombre d'articulations du robot. */
  static const int articulationsNb; // 6

  /** Nom du fichier ou est rangee la liste de parametres chez Fabien. */
  static const char * const PARAMETRES_AFMA6_FILENAME ;

  /** Nom du fichier de parametres MPI de la cam Dragonfly2 DR2-COL. */
  static const char * const CONST_MPI_DRAGONFLY2_WITHOUT_DISTORTION ;
  static const char * const CONST_MPI_DRAGONFLY2_WITH_DISTORTION ;

  /** Nom du fichier ou est rangee la liste de parametres intrinsèque de camera.*/
  static const char * const PARAMETRES_CAMERA_AFMA6_FILENAME ;

  /** Nom de la cam IEEE1394. */
  static const char * const CONST_LABEL_DRAGONFLY2 ;

  /** Differentes cameras installees sur l'afma6. */
  enum CameraRobotType
    {
      CAMERA_DRAGONFLY2_12MM,
    } ;
  /** Camera utilisee par default. */
  static const CameraRobotType defaultCameraRobot;// = CAMERA_DRAGONFLY2_12MM;

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
  void init (const char * filename,
				    const char * filenameMPI);

  /* \brief Lecture des fichiers de configuration.   */
  void init (vpAfma6::CameraRobotType camera, bool usedistortion = false);

  /** \brief Initialisation a l'aide du fichier par default.    */
  void init (void);

  /** \brief Affichage.
   *
   * Affichage de tous les champs de la classe en mode texte.
   */
  friend std::ostream & operator << (std::ostream & os,
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

  /** Lecture d'un fichier de config. */
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
  double  rho;
  /** Camera courante utilisée */
  CameraRobotType camera_current;

public: /* Methodes publiques */

  /* --- VALEURS ---------------------------------------------------------- */

  /** @name Valeurs des butees */
  //@{
  void getJointLimitsMin (vpColVector &qmin) const;
  void getJointLimitsMax (vpColVector &qmax) const;


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
				   vpColVector &jointPos) const;
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

  int computeMGI (const vpHomogeneousMatrix & oMc, vpColVector & q, 
		  bool nearest=true);

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
  //!get the current used camera
  CameraRobotType getCameraRobotType(){return camera_current;};
  /** Place the current used camera*/
  void setCameraRobotType(vpAfma6::CameraRobotType camera){camera_current = camera;};

  void getCameraParameters(vpCameraParameters &cam,
        const unsigned int image_width,
        const unsigned int image_height);
  void getCameraParameters(vpCameraParameters &cam,
        const vpImage<unsigned char> &I);
  void getCameraParameters(vpCameraParameters &cam, const vpImage<vpRGBa> &I);
};



#endif /* #ifndef __AFMA6_H */





/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */

#endif
