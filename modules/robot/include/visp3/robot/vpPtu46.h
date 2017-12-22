/****************************************************************************
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2017 by Inria. All rights reserved.
 *
 * This software is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ViSP with software that can not be combined with the GNU
 * GPL, please contact Inria about acquiring a ViSP Professional
 * Edition License.
 *
 * See http://visp.inria.fr for more information.
 *
 * This software was developed at:
 * Inria Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 *
 * If you have questions regarding the use of this file, please contact
 * Inria at visp@inria.fr
 *
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Description:
 * Interface for the ptu-46 robot.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/

#include <visp3/core/vpConfig.h>

#ifndef __vpPtu46_H
#define __vpPtu46_H

/* ----------------------------------------------------------------------- */
/* --- INCLUDES -------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/* --- GENERAL --- */
#include <iostream>

/* --- ViSP --- */
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpMatrix.h>
#include <visp3/core/vpPoseVector.h>

#include <visp3/core/vpMath.h>
#include <visp3/core/vpRxyzVector.h>
#include <visp3/core/vpTranslationVector.h>
#include <visp3/core/vpVelocityTwistMatrix.h>

/*!

  \class vpPtu46

  \ingroup group_robot_real_ptu

  \brief Jacobian, geometric model functionnalities... for ptu46, pan, tilt
  head from Directed Perception.

  See http://www.DPerception.com for more details.

*/

class VISP_EXPORT vpPtu46
{

public: /* Constants */
  /** Nombre d'articulations du robot. */
  static const unsigned int ndof; /*!< Number of dof */

  /** Geometric model */
  static const float L;
  static const float h;

public: /* Methodes publiques */
  vpPtu46(void);
  /*! Destructor that does nothing. */
  virtual ~vpPtu46(){};

  /** @name Inherited functionalities from vpPtu46 */
  //@{
  void init(void);

  void computeMGD(const vpColVector &q, vpHomogeneousMatrix &fMc) const;
  vpHomogeneousMatrix computeMGD(const vpColVector &q) const;
  void computeMGD(const vpColVector &q, vpPoseVector &r) const;

  void get_cMe(vpHomogeneousMatrix &_cMe) const;
  void get_cVe(vpVelocityTwistMatrix &_cVe) const;
  void get_eJe(const vpColVector &q, vpMatrix &eJe) const;
  void get_fJe(const vpColVector &q, vpMatrix &fJe) const;

  //@}
  friend VISP_EXPORT std::ostream &operator<<(std::ostream &os, const vpPtu46 &constant);
};

#endif
