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
 * Exponential map.
 *
 * Authors:
 * Fabien Spindler
 * Francois Chaumette
 *
 *****************************************************************************/

/*!
  \file vpExponentialMap.h
  \brief Provides exponential map computation
*/

#ifndef vpExponentialMap_h
#define vpExponentialMap_h

#include <visp3/core/vpColVector.h>
#include <visp3/core/vpHomogeneousMatrix.h>

/*!
  \class vpExponentialMap

  \ingroup group_core_transformations

  \brief Direct or inverse exponential map computation.

  The exponential map is the relationship between the velocity of a moving
  body and its pose. The exponential map transforms exponentially the velocity
  skew vector \f$ \bf v \f$ applied during a given time \f$\Delta t\f$ to its
  corresponding pose. The exponential map is usually written using homogeneous
  matrices as:

  \f[ {\bf M}_{t+\Delta t} = {\bf M}_{t} \exp^{({\bf v}, \Delta t)} \f]
  where \f${\bf M}_{t}\f$ is a pose before applied velocity and
  \f${\bf M}_{t+1}\f$ the result.

  This class allows to compute the direct or the inverse exponential map.

  - The direct exponential map allows to determine the displacement
    \f$ \exp^{({\bf v}, \Delta t)} \f$ from a velocity vector skew \f$ \bf v
  \f$ applied during a sampling time \f$\Delta t\f$. With direct() the
  sampling time is set to 1 second. With direct(const vpColVector &, const
  double &) the sampling time can be set to an other value where the second
  argument is \f$ \Delta t \f$.

  - The inverse exponential map allows to compute a velocity skew vector \f$
  \bf v \f$ from a displacement measured during a time interval \f$ \Delta t
  \f$. With inverse() the time interval also called sampling time is set to 1
  second. With inverse(const vpHomogeneousMatrix &, const double &) the
  sampling time can be set to an other value where the second argument is \f$
  \Delta t \f$.

  The displacement is represented as an homogeneous matrix implemented in
  vpHomogeneousMatrix. Velocities \f$ \bf v \f$ are represented as a
  velocity skew 6 dimension vector \f$ [v, \omega] \f$, where \f$ v \f$
  is a velocity translation vector with values in m/s and \f$ \omega \f$ a
  velocity rotation vector with values expressed in rad/s.

*/
class VISP_EXPORT vpExponentialMap
{
public:
  static vpHomogeneousMatrix direct(const vpColVector &v);
  static vpHomogeneousMatrix direct(const vpColVector &v, const double &delta_t);
  static vpColVector inverse(const vpHomogeneousMatrix &M);
  static vpColVector inverse(const vpHomogeneousMatrix &M, const double &delta_t);
};
#endif
