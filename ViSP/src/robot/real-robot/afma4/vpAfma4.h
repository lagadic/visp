/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet LAGADIC / IRISA, 2005
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
 *  $Id: vpAfma4.h,v 1.2 2006-04-19 09:01:21 fspindle Exp $
 *
 * Description
 * ============
 *
 * Definition de la classe vpAfma4
 *
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

#include <visp/vpConfig.h>

#ifdef VISP_HAVE_AFMA4

#ifndef __vpAFMA4_H
#define __vpAFMA4_H


/* ----------------------------------------------------------------------- */
/* --- INCLUDES -------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/* --- GENERAL --- */
#include <iostream>                /* Definition class ostream.               */

/* --- ViSP --- */
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpPoseVector.h>

#include <visp/vpMath.h>
#include <visp/vpRxyzVector.h>
#include <visp/vpTranslationVector.h>
#include <visp/vpTwistMatrix.h>


/* ----------------------------------------------------------------------- */
/* --- CLASSE ------------------------------------------------------------ */
/* ----------------------------------------------------------------------- */

/** \brief Lecture du fichiers de config du robot Afma4 et fonctionnalites en
 * decoulant (calcul du jacobien, du MGD, ...).
 *
 */

class vpAfma4
{

public: /* Constantes */

  /** Nombre d'articulations du robot. */
  static const int articulationsNb; // 4

  /** Nom du fichier ou est rangee la liste de parametres chez Fabien. */
  static const char * const PARAMETRES_AFMA4_FILENAME ;

public: /* Attributs publiques */

  float Kp [4]; /* Gains proportionnels */
  float Kd [4]; /* Gains derives */
  float Ki [4]; /* Gains integraux */

  /** Bornes inferieures des intervalles du domaine articulaire,
   * unite 'top' (i.e. un pas du moteur). */
  long QMax [4];           /* En mm et rad. */
  /** Bornes inferieures des intervalles du domaine articulaire,
   * unite 'top' (i.e. un pas du moteur). */
  long QMin [4];           /* En mm et rad. */
  /** Taille d'un top, l'unite de mesure des intervalles articulaires,
   * unite mm et rad. */
  float Unit[4];	/* Rapport UTC et mm/degres */
  float top [4];		/* Rapport UTC et mm/radians */

  long RstQm [4];/* Initialisation codeurs */
  long SensDep [4];/* Sens deplacement consigne positive */
  long EpsMax [4];/* Valeur erreur ou integrateur rentre */
  long TiMax [4];	/* Saturation termes integraux */
  long AccMax [4];/* Acceleration maximale */
  long VitMax [4];/* Vitesses maximales */
  long ErrTMax [4];     /* Erreur trainage maximales */

  float l;		/* Deport suivant x et y des axes 3 et	*/
  float L;		/* 4, par rapport à l'axe 1.		*/
  float regle;		/* Position lue sur la regle graduee.	*/

  /** Vecteur de representation de la rotation entre
   * les reperes poignet R5 et image (R6). C'est la partie rotation
   * de la matrice RPI (unite rad). */
  vpRxyzVector                rrpi;
  /** Vecteur de translation entre les reperes poignet (R6) et image (R7).
   * C'est la partie translation de la matrice RPI (unite mm).*/
  vpTranslationVector         trpi;

  /** Bornes superieures des intervalles du domaine articulaire,
   * unite mm et rad. */
  double                      jointMax [4];       /* En m  et rad. */
  /** Bornes inferieures des intervalles du domaine articulaire,
   * unite mm et rad. */
  double                      jointMin [4];       /* En m  et rad. */
  /** Centre des intervalles du domaine articulaire, unite mm et rad. */
  double                      jointMiddle [4];    /* En m  et rad. */

  /** Matrice homogene de passage entre le poignet (axe 6) et l'image
   * (~ axe 7), i.e. matrice r67. */
  vpHomogeneousMatrix          rpi;                /* En m  et rad. */

  bool                        FlagMod;
  bool                        FlagReset;


  /** Vrai ssi la classe a deja ete initiliasee une fois. */
  bool                        dejaInitialisee;

public: /* Methodes publiques */

  /** \brief Constructeur vide. */
  vpAfma4 (void);

  /** \brief Lecture des fichiers de configuration.   */
  void                        init (const char * paramAfma4);
  /** \brief Initialisation a l'aide du fichier par default.    */
  void                        init (void);
  /** \brief Affichage.
   *
   * Affichage de tous les champs de la classe en mode texte.
   */
  friend ostream & operator << (ostream & os,
				const vpAfma4 & constant);

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


public: /* Methodes publiques */

  /* --- VALEURS ---------------------------------------------------------- */

  /** @name Valeurs des butees */
  //@{

  /** \brief Renvoie les butees basses du robot.   */
  void getJointLimitsMin (double jointMin[4]) const;
  //void getJointLimitsMin (double jointMin[articulationsNb]) const;

  /** \brief Renvoie les butees hautes du robot.    */
  void getJointLimitsMax (double jointMax[4]) const;
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
				   double jointPos[4]) const;
  //void getPositionInJointInterval (const vpColVector &q,
  //                                 double jointPos[articulationsNb]) const;

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



#endif /* #ifndef __AFMA4_H */





/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */

#endif
