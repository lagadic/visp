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
 * Basic moment descriptor
 *
 * Authors:
 * Filip Novotny
 *
 *****************************************************************************/

#include <visp3/core/vpMomentBasic.h>
#include <visp3/core/vpMomentObject.h>
/*!
  Default constructor.
*/
vpMomentBasic::vpMomentBasic() : vpMoment() {}

/*!
  Retrieve all moments of all orders computed. vpMomentBasic::get()[j*order+i]
  refers to moment \f$m_{ij}\f$. \return all computed moments.

  Same behaviour as vpMomentObject.
*/
const std::vector<double> &vpMomentBasic::get() const { return getObject().get(); }

/*!
  Gets the desired moment using indexes.
  \param i : first index of the 2D moment.
  \param j : second index of the 2D moment.
  \return \f$m_{ij}\f$ moment.

  Same behaviour as vpMomentObject.
*/
double vpMomentBasic::get(unsigned int i, unsigned int j) const { return getObject().get(i, j); }

/*!
  Dummy function. Everything is already done in object.
*/
void vpMomentBasic::compute() {}

/*!
  Outputs the moment's values to a stream.
  Same output as in vpMomentObject.
*/
VISP_EXPORT std::ostream &operator<<(std::ostream &os, const vpMomentBasic &m)
{
  os << (__FILE__) << std::endl;
  vpMomentObject::printWithIndices(m.getObject(), os);
  return os;
}

/*!
No dependencies on other vpMoments, since basic moments are computed in
vpMomentObject Just prints the basic moments in vpMomentObject with indices
*/
void vpMomentBasic::printDependencies(std::ostream &os) const
{
  os << (__FILE__) << std::endl;
  os << "No dependencies on other vpMoments, since basic moments are "
        "computed in vpMomentObject"
     << std::endl;
  vpMomentObject::printWithIndices(getObject(), os);
}
