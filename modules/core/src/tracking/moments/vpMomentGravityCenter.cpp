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
 * 2D Gravity Center moment descriptor (usually described by the pair Xg,Yg)
 *
 * Authors:
 * Filip Novotny
 *
 *****************************************************************************/

#include <visp3/core/vpMomentGravityCenter.h>
#include <visp3/core/vpMomentObject.h>
/*!
  Computes the two gravity center coordinates commonly called \f$x_g\f$ and
  \f$y_g\f$.
*/
void vpMomentGravityCenter::compute()
{
  values[0] = getObject().get(1, 0) / getObject().get(0, 0);
  values[1] = getObject().get(0, 1) / getObject().get(0, 0);
}

/*!
  Default constructor.
*/
vpMomentGravityCenter::vpMomentGravityCenter() : vpMoment() { values.resize(2); }

/*!
  Returns a vector of the two gravity center coordinates.
  \return Coordinates in the following moment: \f$(x_g,y_g)\f$.
*/
const std::vector<double> &vpMomentGravityCenter::get() const { return values; }

/*!
  Outputs the moment's values to a stream.
*/
VISP_EXPORT std::ostream &operator<<(std::ostream &os, const vpMomentGravityCenter &m)
{
  os << (__FILE__) << std::endl;
  os << "(Xg,Yg) = (" << m.values[0] << ", " << m.values[1] << ")" << std::endl;
  return os;
}

/*!
Prints its dependencies
Basic moments m10, m01 and m00 from vpMomentObject
*/
void vpMomentGravityCenter::printDependencies(std::ostream &os) const
{
  os << (__FILE__) << std::endl;
  os << "m10 = " << getObject().get(1, 0) << "\t";
  os << "m00 = " << getObject().get(0, 1) << "\t";
  os << "m00 = " << getObject().get(0, 0) << std::endl;
}
