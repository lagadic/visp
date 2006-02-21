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
 *  $Id: vpPtu46.h,v 1.3 2006-02-21 11:14:43 fspindle Exp $
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
#include <iostream>

/* --- ViSP --- */
#include <visp/vpMatrix.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpPoseVector.h>

#include <visp/vpMath.h>
#include <visp/vpRxyzVector.h>
#include <visp/vpTranslationVector.h>
#include <visp/vpTwistMatrix.h>

/*!

  \brief Jacobian, geometric model functionnalities... for ptu46, pan, tilt
  head from Directed Perception.

  See http://www.DPerception.com for more details.

*/


class vpPtu46
{

public: /* Constants */

  /** Nombre d'articulations du robot. */
  static const int   ndof;  /*!< Number of dof */

  /** Geometric model */
  static const float L;
  static const float h;


public: /* Methodes publiques */

  vpPtu46 (void);
  void init (void);

  void  computeMGD (const vpColVector &q, vpHomogeneousMatrix & fMc);

  vpHomogeneousMatrix computeMGD (const vpColVector & q);
  void  computeMGD (const vpColVector & q,  vpPoseVector & r);

  void get_cMe(vpHomogeneousMatrix &_cMe) ;
  void get_cVe(vpTwistMatrix &_cVe) ;
  void get_eJe(const vpColVector &q, vpMatrix &eJe);
  void get_fJe(const vpColVector &q, vpMatrix &fJe);

  friend ostream & operator << (ostream & os,
				const vpPtu46 & constant);
};




/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */

#endif
