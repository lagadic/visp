/*
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2023 by Inria. All rights reserved.
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
 * See https://visp.inria.fr for more information.
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
 * Just the area m00 = mu00
 */
#ifndef _vpMomentArea_h_
#define _vpMomentArea_h_

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpMoment.h>

BEGIN_VISP_NAMESPACE
class vpMomentObject;
class vpMomentCentered; // Required for discrete case of vpMomentObject

/*!
 * \class vpMomentArea
 *
 * \ingroup group_core_moments
 *
 * \brief Class handling the surface moment.
 *
 * For a dense planar object, the area corresponds to the zero-order moment:
 * \f[ a = m_{00} = \mu_{00} \f]
 *
 * When considering a discrete set of points, the moment \f$ m_{00} \f$ simply
 * corresponds to the number of points. Since this is of no use in a servoing
 * scheme, this class uses in this case \f$ a = \mu_{20} + \mu_{02} \f$, which is
 * invariant to planar translation and rotation.
*/
class VISP_EXPORT vpMomentArea : public vpMoment
{
public:
  vpMomentArea();

  /** @name Inherited functionalities from vpMomentArea */
  //@{
  void compute();
  //! Moment name.
  const std::string name() const { return "vpMomentArea"; }
  void printDependencies(std::ostream &os) const;
  //@}
  friend VISP_EXPORT std::ostream &operator<<(std::ostream &os, const vpMomentArea &m);
};
END_VISP_NAMESPACE
#endif
