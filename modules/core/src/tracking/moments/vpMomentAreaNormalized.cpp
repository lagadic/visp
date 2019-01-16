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
 * 2D normalized surface moment descriptor (usually described as An)
 *
 * Authors:
 * Filip Novotny
 *
 *****************************************************************************/
#include <cmath>
#include <visp3/core/vpMomentAreaNormalized.h>
#include <visp3/core/vpMomentCentered.h>
#include <visp3/core/vpMomentDatabase.h>
#include <visp3/core/vpMomentObject.h>

/*!
  Computes the normalized area \f$ a_n=Z^* \sqrt{\frac{a^*}{a}} \f$.
  Depends on vpMomentCentered.
*/
void vpMomentAreaNormalized::compute()
{
  bool found_moment_centered;

  /* getMoments() returns a reference to a vpMomentDatabase. (a protected
    member inherited from vpMoment)
    .get() 		is a member function of vpMomentDatabase that returns
    a specific moment which is linked to it*/
  const vpMomentCentered &momentCentered =
      static_cast<const vpMomentCentered &>(getMoments().get("vpMomentCentered", found_moment_centered));

  if (!found_moment_centered)
    throw vpException(vpException::notInitialized, "vpMomentCentered not found");

  double a;
  /* getObject() returns a reference to the vpMomentObject from which
   * the moment values are calculated. (public member of vpMoment)*/
  if (getObject().getType() == vpMomentObject::DISCRETE)
    a = momentCentered.get(2, 0) + momentCentered.get(0, 2);
  else
    a = getObject().get(0, 0);

  values[0] = desiredDepth * sqrt(desiredSurface / a);
}

/*!
  Default constructor.
  \param a_star : desired area \e a* when the visual servoing converges.
  \param Z_star : desired depth \e Z* when the visual servoing converges.
*/
vpMomentAreaNormalized::vpMomentAreaNormalized(double a_star, double Z_star)
  : vpMoment(), desiredSurface(a_star), desiredDepth(Z_star)
{
  values.resize(1);
}

/*!
  Outputs the moment's values to a stream.
*/
VISP_EXPORT std::ostream &operator<<(std::ostream &os, const vpMomentAreaNormalized &m)
{
  os << (__FILE__) << std::endl;
  os << "An = " << m.values[0] << std::endl;
  return os;
}

/*!
Prints dependencies namely,
1. Depth at desired pose Z*
2. Area moment at desired pose
   m00* if DENSE moment object, (mu20* + mu02*) if DISCRETE moment object
3. Area moment at current pose
   m00 if DENSE moment object, (mu20 + mu02) if DISCRETE moment object
*/
void vpMomentAreaNormalized::printDependencies(std::ostream &os) const
{
  os << (__FILE__) << std::endl;
  os << "Desired depth Z* = " << desiredDepth << std::endl;
  os << "Desired area m00* = " << desiredSurface << std::endl;

  bool found_moment_centered;
  const vpMomentCentered &momentCentered =
      static_cast<const vpMomentCentered &>(getMoments().get("vpMomentCentered", found_moment_centered));
  if (!found_moment_centered)
    throw vpException(vpException::notInitialized, "vpMomentCentered not found");

  double a;
  if (getObject().getType() == vpMomentObject::DISCRETE)
    a = momentCentered.get(2, 0) + momentCentered.get(0, 2);
  else
    a = getObject().get(0, 0);
  os << "a = " << a << std::endl;
}
