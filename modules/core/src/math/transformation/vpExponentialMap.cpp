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

#include <visp3/core/vpExponentialMap.h>

BEGIN_VISP_NAMESPACE
/*!

  Compute the exponential map. The inverse function is inverse().  The
  sampling time is here set to 1 second. To use an other value you should use
  direct(const vpColVector &, const double &).

  \param v : Instantaneous velocity skew represented by a 6 dimension
  vector \f$ {\bf v} = [v, \omega] \f$ where \f$ v \f$ is a translation
  velocity vector and \f$ \omega \f$ is a rotation velocity vector.

  \return An homogeneous matrix \f$ \bf M \f$ computed from an instantaneous
  velocity \f$ \bf v \f$, where \f${\bf M} = \exp{({\bf v})} \f$ is the
  displacement of the object when the velocity \f$ \bf v \f$ is applied during
  1 second.

  \sa inverse(const vpHomogeneousMatrix &)
*/
vpHomogeneousMatrix vpExponentialMap::direct(const vpColVector &v) { return vpExponentialMap::direct(v, 1.0); }

/*!

  Compute the exponential map. The inverse function is inverse().

  \param v : Instantaneous velocity skew represented by a 6 dimension
  vector \f$ {\bf v} = [v, \omega] \f$ where \f$ v \f$ is a translation
  velocity vector and \f$ \omega \f$ is a rotation velocity vector.

  \param delta_t : Sampling time \f$ \Delta t \f$. Time during which the
  velocity \f$ \bf v \f$ is applied.

  \return An homogeneous matrix \f$ \bf M \f$ computed from an instantaneous
  velocity \f$ \bf v \f$, where \f${\bf M} = \exp{({\bf v})} \f$ is
  the displacement of the object when the velocity \f$ \bf v \f$ is applied
  during \f$\Delta t\f$ seconds.

  \sa inverse(const vpHomogeneousMatrix &, const double &)
*/
vpHomogeneousMatrix vpExponentialMap::direct(const vpColVector &v, const double &delta_t)
{
  const unsigned int v_size = 6;
  const unsigned int index_0 = 0;
  const unsigned int index_1 = 1;
  const unsigned int index_2 = 2;
  const unsigned int index_3 = 3;
  const unsigned int index_4 = 4;
  const unsigned int index_5 = 5;

  if (v.size() != v_size) {
    throw(vpException(vpException::dimensionError,
                      "Cannot compute direct exponential map from a %d-dim velocity vector. Should be 6-dim.",
                      v.size()));
  }
  double theta, si, co, sinc, mcosc, msinc;
  vpThetaUVector u;
  vpRotationMatrix rd;
  vpTranslationVector dt;

  vpColVector v_dt = v * delta_t;

  u[index_0] = v_dt[index_3];
  u[index_1] = v_dt[index_4];
  u[index_2] = v_dt[index_5];
  rd.build(u);

  theta = sqrt((u[index_0] * u[index_0]) + (u[index_1] * u[index_1]) + (u[index_2] * u[index_2]));
  si = sin(theta);
  co = cos(theta);
  sinc = vpMath::sinc(si, theta);
  mcosc = vpMath::mcosc(co, theta);
  msinc = vpMath::msinc(si, theta);

  dt[index_0] = ((v_dt[index_0] * (sinc + (u[index_0] * u[index_0] * msinc))) +
          (v_dt[index_1] * ((u[index_0] * u[index_1] * msinc) - (u[index_2] * mcosc)))) +
    (v_dt[index_2] * ((u[index_0] * u[index_2] * msinc) + (u[index_1] * mcosc)));

  dt[index_1] = ((v_dt[index_0] * ((u[index_0] * u[index_1] * msinc) + (u[index_2] * mcosc))) +
          (v_dt[index_1] * (sinc + (u[index_1] * u[index_1] * msinc)))) +
    (v_dt[index_2] * ((u[index_1] * u[index_2] * msinc) - (u[index_0] * mcosc)));

  dt[index_2] = ((v_dt[index_0] * ((u[index_0] * u[index_2] * msinc) - (u[index_1] * mcosc))) +
          (v_dt[index_1] * ((u[index_1] * u[index_2] * msinc) + (u[index_0] * mcosc)))) +
    (v_dt[index_2] * (sinc + (u[index_2] * u[index_2] * msinc)));

  vpHomogeneousMatrix Delta;
  Delta.insert(rd);
  Delta.insert(dt);

  return Delta;
}

/*!

  Computes an instantaneous velocity skew from an homogeneous matrix. The
  inverse function is the exponential map, see direct().

  \param M : An homogeneous matrix corresponding to the displacement of an
  object during 1 second.

  \return Instantaneous velocity skew \f$ \bf v \f$ represented by a 6
  dimension vector \f$ [v, \omega] \f$ where \f$ v \f$ is a translation
  velocity vector and \f$ \omega \f$ is a rotation velocity vector.

  \sa direct(const vpColVector &)
*/
vpColVector vpExponentialMap::inverse(const vpHomogeneousMatrix &M) { return vpExponentialMap::inverse(M, 1.0); }

/*!

  Compute an instantaneous velocity from an homogeneous matrix. The inverse
  function is the exponential map, see direct().

  \param M : An homogeneous matrix corresponding to the displacement of an
  object during \f$\Delta t\f$ seconds.

  \param delta_t : Sampling time \f$ \Delta t \f$. Time during which the
  displacement is applied.

  \return Instantaneous velocity skew \f$ \bf v \f$ represented by a 6
  dimension vector \f$ [v, \omega] \f$ where \f$ v \f$ is a translation
  velocity vector and \f$ \omega \f$ is a rotation velocity vector.

  \sa direct(const vpColVector &, const double &)
*/
vpColVector vpExponentialMap::inverse(const vpHomogeneousMatrix &M, const double &delta_t)
{
  vpColVector v(6);
  unsigned int i;
  double theta, si, co, sinc, mcosc, msinc, det;
  vpThetaUVector u;
  vpRotationMatrix Rd, a;
  vpTranslationVector dt;
  const unsigned int index_0 = 0;
  const unsigned int index_1 = 1;
  const unsigned int index_2 = 2;
  const unsigned int index_3 = 3;

  M.extract(Rd);
  u.build(Rd);
  for (i = 0; i < 3; ++i) {
    v[3 + i] = u[i];
  }

  theta = sqrt((u[index_0] * u[index_0]) + (u[index_1] * u[index_1]) + (u[index_2] * u[index_2]));
  si = sin(theta);
  co = cos(theta);
  sinc = vpMath::sinc(si, theta);
  mcosc = vpMath::mcosc(co, theta);
  msinc = vpMath::msinc(si, theta);

  // a below is not a pure rotation matrix, even if not so far from
  // the Rodrigues formula : sinc I + (1-sinc)/t^2 VV^T + (1-cos)/t^2 [V]_X
  // with V = t.U

  a[index_0][index_0] = sinc + (u[index_0] * u[index_0] * msinc);
  a[index_0][index_1] = (u[index_0] * u[index_1] * msinc) - (u[index_2] * mcosc);
  a[index_0][index_2] = (u[index_0] * u[index_2] * msinc) + (u[index_1] * mcosc);

  a[index_1][index_0] = (u[index_0] * u[index_1] * msinc) + (u[index_2] * mcosc);
  a[index_1][index_1] = sinc + (u[index_1] * u[index_1] * msinc);
  a[index_1][index_2] = (u[index_1] * u[index_2] * msinc) - (u[index_0] * mcosc);

  a[index_2][index_0] = (u[index_0] * u[index_2] * msinc) - (u[index_1] * mcosc);
  a[index_2][index_1] = (u[index_1] * u[index_2] * msinc) + (u[index_0] * mcosc);
  a[index_2][index_2] = sinc + (u[index_2] * u[index_2] * msinc);

  det = (((((a[index_0][index_0] * a[index_1][index_1] * a[index_2][index_2]) + (a[index_1][index_0] * a[index_2][index_1] * a[index_0][index_2])) + (a[index_0][index_1] * a[index_1][index_2] * a[index_2][index_0])) -
          (a[index_2][index_0] * a[index_1][index_1] * a[index_0][index_2])) - (a[index_1][index_0] * a[index_0][index_1] * a[index_2][index_2])) - (a[index_0][index_0] * a[index_2][index_1] * a[index_1][index_2]);

  if (fabs(det) > 1.e-5) {
    v[index_0] = ((((((M[index_0][index_3] * a[index_1][index_1] * a[index_2][index_2]) + (M[index_1][index_3] * a[index_2][index_1] * a[index_0][index_2])) + (M[index_2][index_3] * a[index_0][index_1] * a[index_1][index_2])) -
                    (M[index_2][index_3] * a[index_1][index_1] * a[index_0][index_2])) - (M[index_1][index_3] * a[index_0][index_1] * a[index_2][index_2])) - (M[index_0][index_3] * a[index_2][index_1] * a[index_1][index_2])) /
      det;
    v[index_1] = ((((((a[index_0][index_0] * M[index_1][index_3] * a[index_2][index_2]) + (a[index_1][index_0] * M[index_2][index_3] * a[index_0][index_2])) + (M[index_0][index_3] * a[index_1][index_2] * a[index_2][index_0])) -
                    (a[index_2][index_0] * M[index_1][index_3] * a[index_0][index_2])) - (a[index_1][index_0] * M[index_0][index_3] * a[index_2][index_2])) - (a[index_0][index_0] * M[index_2][index_3] * a[index_1][index_2])) /
      det;
    v[index_2] = ((((((a[index_0][index_0] * a[index_1][index_1] * M[index_2][index_3]) + (a[index_1][index_0] * a[index_2][index_1] * M[index_0][index_3])) + (a[index_0][index_1] * M[index_1][index_3] * a[index_2][index_0])) -
                    (a[index_2][index_0] * a[index_1][index_1] * M[index_0][index_3])) - (a[index_1][index_0] * a[index_0][index_1] * M[index_2][index_3])) - (a[index_0][index_0] * a[index_2][index_1] * M[index_1][index_3])) /
      det;
  }
  else {
    v[index_0] = M[index_0][index_3];
    v[index_1] = M[index_1][index_3];
    v[index_2] = M[index_2][index_3];
  }

  // Apply the sampling time to the computed velocity
  v /= delta_t;

  return v;
}
END_VISP_NAMESPACE
