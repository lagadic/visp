/*
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2024 by Inria. All rights reserved.
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
 * Exponential map.
 */

/*!
  \file vpExponentialMap.h
  \brief Provides exponential map computation
*/

#ifndef VP_EXPONENTIAL_MAP_H
#define VP_EXPONENTIAL_MAP_H

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpColVector.h>
#include <visp3/core/vpHomogeneousMatrix.h>

BEGIN_VISP_NAMESPACE
/*!
  \class vpExponentialMap

  \ingroup group_core_transformations

  \brief Direct or inverse exponential map computation.

  The exponential map gives the relationship between the velocity of a moving
  body and its displacement:

  \f[ \exp({^c}{\bf v}_c(t - \Delta t)) = {^{c(t - \Delta t)}}{\bf M}_{c(t)} \f]

  where \f$ {^c}{\bf v}_c(t - \Delta t)\f$ is the velocity skew vector at the previous iteration applied during \f$\Delta t\f$
  seconds at point \f$ c \f$ in frame \f$ c \f$, while \f$ {^{c(t- \Delta t)}}{\bf M}_{c(t)} \f$
  is the corresponding displacement.

  This class allows to compute the direct or the inverse exponential map.

  - The direct exponential map allows to compute the displacement
    \f${^{c(t - \Delta t)}}{\bf M}_{c(t)}\f$ using \f${^c}{\bf v}_c(t - \Delta t)\f$ as input:
    \f[ {^{o}}{\bf M}_{c(t)} = {^{o}}{\bf M}_{c(t - \Delta t)} \exp({^c}{\bf v}_c(t - \Delta t)) \f]
    where \f$ o \f$ is a reference frame.
    With direct(), the velocity skew vector \f$ {^c}{\bf v}_c(t - \Delta t) \f$ is applied during 1 second
    considering \f$ \Delta t = 1\f$. With direct(const vpColVector &, const double &)
    the sampling time can be set to an other value where the second
    argument is \f$ \Delta t \f$.

  - The inverse exponential map allows to compute the velocity skew vector \f$
    {^c}{\bf v}_c(t - \Delta t) \f$ from the displacement \f$ {^{c(t - \Delta t)}}{\bf M}_{c(t)}\f$
    measured during a time interval \f$ \Delta t \f$. With inverse() the time interval
    also called sampling time is set to 1 second. With
    inverse(const vpHomogeneousMatrix &, const double &) the sampling time can
    be set to an other value where the second
    argument is \f$ \Delta t \f$.

  A displacement \f$ \bf M \f$ is represented as an homogeneous matrix implemented in
  vpHomogeneousMatrix. A velocity \f$ \bf v \f$ is represented as a
  6 dimension velocity skew vector \f$ [v, \omega] \f$, where \f$ v \f$
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
END_VISP_NAMESPACE
#endif
