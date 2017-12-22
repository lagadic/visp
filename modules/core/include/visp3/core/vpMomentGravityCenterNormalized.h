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
 * 2D normalized gravity center moment descriptor (usually described by the
 *pair Xn,Yn)
 *
 * Authors:
 * Filip Novotny
 *
 *****************************************************************************/
/*!
  \file vpMomentGravityCenterNormalized.h
  \brief 2D normalized gravity center moment descriptor (usually described by
  the pair Xn,Yn)
*/
#ifndef __MOMENTGRAVITYNORMALIZED_H__
#define __MOMENTGRAVITYNORMALIZED_H__

#include <visp3/core/vpMomentDatabase.h>
#include <visp3/core/vpMomentGravityCenter.h>
class vpMomentObject;

/*!
  \class vpMomentGravityCenterNormalized

  \ingroup group_core_moments

  \brief Class describing 2D normalized gravity center moment.

  Centered and normalized gravity center moment is defined as follows:
  \f$(x_n,y_n)\f$ where \f$x_n = x_g a_n\f$ and \f$y_n = y_g a_n\f$.

  vpMomentGravityCenterNormalized depends on vpMomentAreaNormalized to get
  access to \f$a_n\f$ and on vpMomentGravityCenter to get access to
  \f$(x_g,y_g)\f$ .
*/
class VISP_EXPORT vpMomentGravityCenterNormalized : public vpMomentGravityCenter
{
public:
  vpMomentGravityCenterNormalized();
  virtual ~vpMomentGravityCenterNormalized(){};
  void compute();
  //! Moment name.
  const char *name() const { return "vpMomentGravityCenterNormalized"; }
  void printDependencies(std::ostream &os) const;
  friend VISP_EXPORT std::ostream &operator<<(std::ostream &os, const vpMomentGravityCenterNormalized &v);
};

#endif
