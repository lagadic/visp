/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet LAGADIC / IRISA, 2006
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      vpBiclops.h
 * Project:   ViSP chapitre robot
 * Author:    Fabien Spindler
 *
 * Version control
 * ===============
 *
 *
 *
 *  $Id: vpBiclops.h,v 1.1 2006-02-21 11:16:11 fspindle Exp $
 *
 * Description
 * ============
 *
 * vpBiclops definition.
 *
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

#include <visp/vpConfig.h>

#ifndef __vpBiclops_H
#define __vpBiclops_H


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

  \brief Jacobian, geometric model functionnalities... for biclops, pan, tilt
  head.

  See http://www.traclabs.com/tracbiclops.htm for more details.

*/

class vpBiclops
{

public: /* Constants */

  static const int   ndof;  /*!< Number of dof */


  /* Geometric model */
  static const float h;

  static const float panJointLimit;
  static const float tiltJointLimit;
  static const float speedLimit;


public:

  vpBiclops (void);
  void init (void);

  void computeMGD (const vpColVector &q, vpHomogeneousMatrix & fMc);

  vpHomogeneousMatrix computeMGD (const vpColVector & q);
  void computeMGD (const vpColVector & q,  vpPoseVector & r);

  void get_cMe(vpHomogeneousMatrix &_cMe) ;
  void get_cVe(vpTwistMatrix &_cVe) ;
  void get_eJe(const vpColVector &q, vpMatrix &eJe);
  void get_fJe(const vpColVector &q, vpMatrix &fJe);

  friend ostream & operator << (ostream & os,
				const vpBiclops & constant);
};




/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */

#endif
