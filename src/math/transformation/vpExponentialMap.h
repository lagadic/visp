/****************************************************************************
 *
 * $Id$
 *
 * Copyright (C) 1998-2006 Inria. All rights reserved.
 *
 * This software was developed at:
 * IRISA/INRIA Rennes
 * Projet Lagadic
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * http://www.irisa.fr/lagadic
 *
 * This file is part of the ViSP toolkit
 *
 * This file may be distributed under the terms of the Q Public License
 * as defined by Trolltech AS of Norway and appearing in the file
 * LICENSE included in the packaging of this file.
 *
 * Licensees holding valid ViSP Professional Edition licenses may
 * use this file in accordance with the ViSP Commercial License
 * Agreement provided with the Software.
 *
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Contact visp@irisa.fr if any conditions of this licensing are
 * not clear to you.
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

#include <visp/vpConfig.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpColVector.h>

/*!
  \class vpExponentialMap

  \ingroup ExpMapTransformation

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
