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
 * Centered moment descriptor
 *
 * Authors:
 * Filip Novotny
 *
 *****************************************************************************/

#include <cassert>
#include <exception>
#include <visp3/core/vpMomentCentered.h>
#include <visp3/core/vpMomentGravityCenter.h>
#include <visp3/core/vpMomentObject.h>

/*!
  To set the values of centred moments. Required when normalizing the moment
  values.
  @param i : first index of the 2D moment.
  @param j : second index of the 2D moment.
  @param value : value of the moment.
*/
void vpMomentCentered::set(unsigned int i, unsigned int j, double value)
{
  vpMomentObject mobj = getObject();
  assert(i + j <= mobj.getOrder());
  if (i + j > mobj.getOrder())
    throw vpException(vpException::badValue, "You cannot set that value.");
  values[j * (mobj.getOrder() + 1) + i] = value;
}

/*!
  Computes centered moments of all available orders.
  Depends on vpMomentGravityCenter.
*/
void vpMomentCentered::compute()
{
  bool found_moment_gravity;
  values.resize((getObject().getOrder() + 1) * (getObject().getOrder() + 1));

  const vpMomentGravityCenter &momentGravity =
      static_cast<const vpMomentGravityCenter &>(getMoments().get("vpMomentGravityCenter", found_moment_gravity));
  if (!found_moment_gravity)
    throw vpException(vpException::notInitialized, "vpMomentGravityCenter not found");

  unsigned int order = getObject().getOrder() + 1;
  for (unsigned int j = 0; j < (order); j++) {
    for (unsigned int i = 0; i < order - j; i++) {
      unsigned int c = order * j + i;
      values[c] = 0;
      for (unsigned int k = 0; k <= i; k++) {
        double Xg_i_k = pow(-momentGravity.get()[0], (int)(i - k));
        double comb_i_k = static_cast<double>(vpMath::comb(i, k));
        for (unsigned int l = 0; l <= j; l++) {
          values[c] += static_cast<double>(comb_i_k * vpMath::comb(j, l) * Xg_i_k *
                                           pow(-momentGravity.get()[1], (int)(j - l)) * getObject().get(k, l));
        }
      }
    }
  }
}

/*!
  Default constructor.
*/
vpMomentCentered::vpMomentCentered() : vpMoment() {}

/*!
  Gets the desired moment using indexes.
  \param i : first index of the centered moment.
  \param j : second index of the centered moment.
  \return \f$\mu_{ij}\f$ moment.
*/
double vpMomentCentered::get(unsigned int i, unsigned int j) const
{
  unsigned int order = getObject().getOrder();
  assert(i + j <= order);
  if (i + j > order)
    throw vpException(vpException::badValue, "The requested value has not "
                                             "been computed, you should "
                                             "specify a higher order.");

  return values[j * (order + 1) + i];
}

/*!
  Outputs the centered moment's values \f$\mu_{ij}\f$ to a stream presented as
a matrix. The first line corresponds to \f$\mu_{0[0:order]}\f$, the second one
to \f$\mu_{1[0:order]}\f$ Values in table corresponding to a higher order are
marked with an "x" and not computed.

  For example, if the maximal order is 3, the following values are provided:

  \code
u00 u10 u20 u30
u01 u11 u21 x
u02 u12  x  x
u30 x    x  x
  \endcode

 This output will be followed by an output with indexes as produced by
printWithIndices() function
*/
VISP_EXPORT std::ostream &operator<<(std::ostream &os, const vpMomentCentered &m)
{
  for (unsigned int i = 0; i < m.values.size(); i++) {
    if (i % (m.getObject().getOrder() + 1) == 0)
      os << std::endl;

    if ((i % (m.getObject().getOrder() + 1) + i / (m.getObject().getOrder() + 1)) < m.getObject().getOrder() + 1)
      os << m.values[i];
    else
      os << "x";

    os << "\t";
  }
  os << std::endl;
  m.printWithIndices(os);
  return os;
}

/*!
Print in a readable form which looks better than output from << operator
*/
void vpMomentCentered::printWithIndices(std::ostream &os) const
{
  unsigned int orderp1 = getObject().getOrder() + 1;
  for (unsigned int k = 0; k < orderp1; k++) {
    for (unsigned int l = 0; l < orderp1 - k; l++) {
      os << "mu[" << k << "," << l << "] = " << this->get(k, l) << "\t";
    }
    os << std::endl;
  }
  os << std::endl;
}

/*!
Prints moments required for calculation of vpMomentCentered,
which are
1. Raw geometric moments (vpMomentObject) and
2. Centre of gravity (vpMomentGravityCentered)
*/
void vpMomentCentered::printDependencies(std::ostream &os) const
{
  os << (__FILE__) << std::endl;
  /*
  Retreive the raw moments
  */
  const vpMomentObject objt = getObject();
  vpMomentObject::printWithIndices(objt, os);

  /*
  Get xg,yg
  */
  bool found_moment_gravity;
  const vpMomentGravityCenter &momentGravity =
      static_cast<const vpMomentGravityCenter &>(getMoments().get("vpMomentGravityCenter", found_moment_gravity));
  if (!found_moment_gravity)
    throw vpException(vpException::notInitialized, "vpMomentGravityCenter not found");
  os << "Xg = " << momentGravity.getXg() << "\t"
     << "Yg = " << momentGravity.getYg() << std::endl;
}
