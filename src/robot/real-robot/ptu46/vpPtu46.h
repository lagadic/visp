/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet LAGADIC / IRISA, 2006
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      vpPtu46.h
 * Project:   ViSP chapitre robot
 * Author:    Fabien Spindler
 *
 * Version control
 * ===============
 *
 *
 *
 *  $Id: vpPtu46.h,v 1.2 2006-01-25 17:30:33 fspindle Exp $
 *
 * Description
 * ============
 *
 * vpPtu46 definition.
 *
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

#include <visp/vpConfig.h>

#ifndef __vpPtu46_H
#define __vpPtu46_H


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

class vpPtu46
{

public: /* Constantes */

  /** Nombre d'articulations du robot. */
  static const int   articulationsNb;

  /** Geometric model */
  static const float L;
  static const float h;
  static const float l2;


public: /* Methodes publiques */

  /** \brief Constructeur vide. */
  vpPtu46 (void);


  /** \brief Initialisation a l'aide du fichier par default.    */
  void                        init (void);

  /** \brief Affichage.
   *
   * Affichage de tous les champs de la classe en mode texte.
   */
  friend ostream & operator << (ostream & os,
				const vpPtu46 & constant);


public:

  //! \brief Calcul le MGD du robot.
  void  computeMGD (const vpColVector &q, vpHomogeneousMatrix & fMc);

  //! \brief Calcul le MGD.
  vpHomogeneousMatrix          computeMGD (const vpColVector & q);
  //! \brief Calcul le MGD du robot sous forme d'un vecteur.
  void  computeMGD (const vpColVector & q,  vpPoseVector & r);

  void get_cMe(vpHomogeneousMatrix &_cMe) ;
  void get_cVe(vpTwistMatrix &_cVe) ;
  void get_eJe(const vpColVector &q, vpMatrix &eJe);
  void get_fJe(const vpColVector &q, vpMatrix &fJe);
};




/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */

#endif
