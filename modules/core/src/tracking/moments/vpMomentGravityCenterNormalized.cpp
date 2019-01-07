/****************************************************************************
 *
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2019 by Inria. All rights reserved.
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

#include <visp3/core/vpMomentAreaNormalized.h>
#include <visp3/core/vpMomentGravityCenter.h>
#include <visp3/core/vpMomentGravityCenterNormalized.h>
#include <visp3/core/vpMomentObject.h>

/*!
  Computes normalized gravity center moment.
  Depends on vpMomentAreaNormalized and on vpMomentGravityCenter.
*/
void vpMomentGravityCenterNormalized::compute()
{
  bool found_moment_gravity;
  bool found_moment_surface_normalized;

  const vpMomentAreaNormalized &momentSurfaceNormalized = static_cast<const vpMomentAreaNormalized &>(
      getMoments().get("vpMomentAreaNormalized", found_moment_surface_normalized));
  const vpMomentGravityCenter &momentGravity =
      static_cast<const vpMomentGravityCenter &>(getMoments().get("vpMomentGravityCenter", found_moment_gravity));

  if (!found_moment_surface_normalized)
    throw vpException(vpException::notInitialized, "vpMomentAreaNormalized not found");
  if (!found_moment_gravity)
    throw vpException(vpException::notInitialized, "vpMomentGravityCenter not found");

  double Xn = momentGravity.get()[0] * momentSurfaceNormalized.get()[0];
  double Yn = momentGravity.get()[1] * momentSurfaceNormalized.get()[0];

  values[0] = Xn;
  values[1] = Yn;
}

/*!
  Default constructor.
*/
vpMomentGravityCenterNormalized::vpMomentGravityCenterNormalized() : vpMomentGravityCenter() {}

/*!
  Outputs the moment's values to a stream.
*/
VISP_EXPORT std::ostream &operator<<(std::ostream &os, const vpMomentGravityCenterNormalized &m)
{
  os << (__FILE__) << std::endl;
  os << "(Xn,Yn) = (" << m.values[0] << ", " << m.values[1] << ")" << std::endl;
  return os;
}

/*!
Prints the dependent moments,
1. centre of gravity
2. normalized area moment
*/
void vpMomentGravityCenterNormalized::printDependencies(std::ostream &os) const
{
  os << (__FILE__) << std::endl;
  bool found_moment_gravity;
  bool found_moment_surface_normalized;

  const vpMomentAreaNormalized &momentSurfaceNormalized = static_cast<const vpMomentAreaNormalized &>(
      getMoments().get("vpMomentAreaNormalized", found_moment_surface_normalized));
  const vpMomentGravityCenter &momentGravity =
      static_cast<const vpMomentGravityCenter &>(getMoments().get("vpMomentGravityCenter", found_moment_gravity));

  if (!found_moment_surface_normalized)
    throw vpException(vpException::notInitialized, "vpMomentAreaNormalized not found");
  if (!found_moment_gravity)
    throw vpException(vpException::notInitialized, "vpMomentGravityCenter not found");
  os << "Xg = " << momentGravity.get()[0] << "\t"
     << "Yg = " << momentGravity.get()[1] << std::endl;
  os << "An = " << momentSurfaceNormalized.get()[0] << std::endl;
}
