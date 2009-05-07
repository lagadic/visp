/****************************************************************************
 *
 * $Id$
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
 * Interface for the ptu-46 robot.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/

#include <visp/vpConfig.h>

#ifndef __vpPtu46_H
#define __vpPtu46_H


/* ----------------------------------------------------------------------- */
/* --- INCLUDES -------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/* --- GENERAL --- */
#include <iostream>

/* --- ViSP --- */
#include <visp/vpConfig.h>
#include <visp/vpMatrix.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpPoseVector.h>

#include <visp/vpMath.h>
#include <visp/vpRxyzVector.h>
#include <visp/vpTranslationVector.h>
#include <visp/vpTwistMatrix.h>

/*!

  \class vpPtu46

  \ingroup Ptu46

  \brief Jacobian, geometric model functionnalities... for ptu46, pan, tilt
  head from Directed Perception.

  See http://www.DPerception.com for more details.

*/


class VISP_EXPORT vpPtu46
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

  friend std::ostream & operator << (std::ostream & os,
				     const vpPtu46 & constant);
};




/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */

#endif
