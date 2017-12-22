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
 * Centered moment descriptor
 *
 * Authors:
 * Filip Novotny
 *
 *****************************************************************************/

#ifndef __MOMENTCENTERED_H__
#define __MOMENTCENTERED_H__

#include <visp3/core/vpMoment.h>
/*!
  \file vpMomentCentered.h
  \brief Centered moment descriptor (also refered to as \f$\mu_{ij}\f$).

*/

class vpMomentObject;

/*!
  \class vpMomentCentered

  \ingroup group_core_moments

  \brief This class defines the double-indexed centered moment descriptor
  \f$\mu_{ij}\f$.

  In the case of a dense object \e O, centered moments are defined by:
  \f[\mu_{ij}= \int \int_{O} (x_k-x_g)^j (y_k-y_g)^j\f]

  In the case of a discrete set of \e n points, centered moments are defined
  by: \f[\mu_{ij}= \sum_{k=1}^{n} (x_k-x_g)^j (y_k-y_g)^j\f]

  where \f$(x_g,y_g)\f$ are the coordinates of the center of gravity.

  The centered moments are computed from the object at the highest possible
  order. For example if the vpMomentObject is defined up to order 5,
  vpMomentCentered will be too.

  Values of vpMomentCentered may be accessed by one of the two
  vpMomentCentered::get methods. When using vpMomentCentered::get (), the
  format of the return vector is the following: \f$ \mu_{ij} \f$ is stored at
  vpMomentCentered::get ()[j* (vpMomentObject::getOrder () +1)+i]

  vpMomentCentered depends on vpMomentGravityCenter.

*/
class VISP_EXPORT vpMomentCentered : public vpMoment
{
public:
  vpMomentCentered();
  virtual ~vpMomentCentered(){};

  void compute();
  double get(unsigned int i, unsigned int j) const;

  inline const std::vector<double> &get() const;
  /*!
     Moment name.
  */
  inline const char *name() const { return "vpMomentCentered"; }

  friend VISP_EXPORT std::ostream &operator<<(std::ostream &os, const vpMomentCentered &v);
  void printWithIndices(std::ostream &os) const;
  void printDependencies(std::ostream &os) const;

protected:
  void set(unsigned int i, unsigned int j, double value);
};

/*!
  Returns all centered moment values \f$\mu_{ij}\f$ with \f$i+j \leq order\f$
where order is the object's order.

  \return Vector of moment values. To access \f$\mu_{ij}\f$, you have to read
vpMomentObject::get()[j*order+i].

  For example, if the maximal order is 3, the following values are provided:

  \code
u00 u10 u20 u01 u11 u21 u02 u12 u12 u30 u03
  \endcode

  To have a better reading of the moments you can picture them as a triangular
  matrix:

  \code
u00 u10 u20 u30 u01 u11 u21 x u02 u12  x  x u30 x    x  x
  \endcode

  The moments of the same order are on each of the matrix reverse diagonals.
  To access for example to the centered moment \f$\mu_{12}\f$, you should use
  this kind of code:

  \code
vpMomentCentered mc;
//[...]
mc.compute();
double mu12;
mu12 = mc.get()[2*(obj.getOrder()+1)+1]; // i=1 and j=2
mu12 = mc.get(1,2); // the same
  \endcode
*/
inline const std::vector<double> &vpMomentCentered::get() const { return vpMoment::get(); }

#endif
