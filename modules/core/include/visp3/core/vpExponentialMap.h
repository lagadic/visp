/****************************************************************************
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2015 by Inria. All rights reserved.
 *
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
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

#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpColVector.h>

/*!
  \class vpExponentialMap

  \ingroup group_core_transformations

  \brief Direct or inverse exponential map computation.

  - The direct exponential map allows to determine a displacement from a
    velocity vector applied during a sampling time. With direct() the sampling
    time is set to 1 second. With direct(const vpColVector &, const float &)
    the sampling time can be set to an other value.

  - The inverse exponential map allows to compute a velocity vector from a
    displacement measured during a time interval. With inverse() the time
    interval also called sampling time is set to 1 second. With
    inverse(const vpHomogeneousMatrix &, const float &) the sampling time
    can be set to an other value.

  The displacement is represented as an homogeneous matrix
  (vpHomogeneousMatrix). Velocities are represented as a \f$ [{\bf t}, {\bf
  \theta u} ]^t \f$ 6 dimension vector where \f$ t \f$ is a translation vector
  (see vpTranslationVector) and \f$ \theta u \f$ a rotation vector (see
  vpThetaUVector).

*/
class VISP_EXPORT vpExponentialMap
{

public:
  static vpHomogeneousMatrix direct(const vpColVector &v);
  static vpHomogeneousMatrix direct(const vpColVector &v,
				    const double &delta_t);
  static vpColVector inverse(const vpHomogeneousMatrix &M);
  static vpColVector inverse(const vpHomogeneousMatrix &M,
			     const double &delta_t);

};
#endif


/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
