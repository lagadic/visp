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
 * Exponential map.
 *
 * Authors:
 * Fabien Spindler
 * Francois Chaumette
 *
 *****************************************************************************/

#include <visp3/core/vpExponentialMap.h>

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
  double theta, si, co, sinc, mcosc, msinc;
  vpThetaUVector u;
  vpRotationMatrix rd;
  vpTranslationVector dt;

  vpColVector v_dt = v * delta_t;

  u[0] = v_dt[3];
  u[1] = v_dt[4];
  u[2] = v_dt[5];
  rd.buildFrom(u);

  theta = sqrt(u[0] * u[0] + u[1] * u[1] + u[2] * u[2]);
  si = sin(theta);
  co = cos(theta);
  sinc = vpMath::sinc(si, theta);
  mcosc = vpMath::mcosc(co, theta);
  msinc = vpMath::msinc(si, theta);

  dt[0] = v_dt[0] * (sinc + u[0] * u[0] * msinc) + v_dt[1] * (u[0] * u[1] * msinc - u[2] * mcosc) +
          v_dt[2] * (u[0] * u[2] * msinc + u[1] * mcosc);

  dt[1] = v_dt[0] * (u[0] * u[1] * msinc + u[2] * mcosc) + v_dt[1] * (sinc + u[1] * u[1] * msinc) +
          v_dt[2] * (u[1] * u[2] * msinc - u[0] * mcosc);

  dt[2] = v_dt[0] * (u[0] * u[2] * msinc - u[1] * mcosc) + v_dt[1] * (u[1] * u[2] * msinc + u[0] * mcosc) +
          v_dt[2] * (sinc + u[2] * u[2] * msinc);

  vpHomogeneousMatrix Delta;
  Delta.insert(rd);
  Delta.insert(dt);

  if (0) // test new version wrt old version
  {
    // old version
    unsigned int i, j;

    double s;
    // double u[3];
    //  vpRotationMatrix rd ;
    //  vpTranslationVector dt ;

    s = sqrt(v_dt[3] * v_dt[3] + v_dt[4] * v_dt[4] + v_dt[5] * v_dt[5]);
    if (s > 1.e-15) {
      for (i = 0; i < 3; i++)
        u[i] = v_dt[3 + i] / s;
      double sinu = sin(s);
      double cosi = cos(s);
      double mcosi = 1 - cosi;
      rd[0][0] = cosi + mcosi * u[0] * u[0];
      rd[0][1] = -sinu * u[2] + mcosi * u[0] * u[1];
      rd[0][2] = sinu * u[1] + mcosi * u[0] * u[2];
      rd[1][0] = sinu * u[2] + mcosi * u[1] * u[0];
      rd[1][1] = cosi + mcosi * u[1] * u[1];
      rd[1][2] = -sinu * u[0] + mcosi * u[1] * u[2];
      rd[2][0] = -sinu * u[1] + mcosi * u[2] * u[0];
      rd[2][1] = sinu * u[0] + mcosi * u[2] * u[1];
      rd[2][2] = cosi + mcosi * u[2] * u[2];

      dt[0] = v_dt[0] * (sinu / s + u[0] * u[0] * (1 - sinu / s)) +
              v_dt[1] * (u[0] * u[1] * (1 - sinu / s) - u[2] * mcosi / s) +
              v_dt[2] * (u[0] * u[2] * (1 - sinu / s) + u[1] * mcosi / s);

      dt[1] = v_dt[0] * (u[0] * u[1] * (1 - sinu / s) + u[2] * mcosi / s) +
              v_dt[1] * (sinu / s + u[1] * u[1] * (1 - sinu / s)) +
              v_dt[2] * (u[1] * u[2] * (1 - sinu / s) - u[0] * mcosi / s);

      dt[2] = v_dt[0] * (u[0] * u[2] * (1 - sinu / s) - u[1] * mcosi / s) +
              v_dt[1] * (u[1] * u[2] * (1 - sinu / s) + u[0] * mcosi / s) +
              v_dt[2] * (sinu / s + u[2] * u[2] * (1 - sinu / s));
    } else {
      for (i = 0; i < 3; i++) {
        for (j = 0; j < 3; j++)
          rd[i][j] = 0.0;
        rd[i][i] = 1.0;
        dt[i] = v_dt[i];
      }
    }
    // end old version

    // Test of the new version
    vpHomogeneousMatrix Delta_old;
    Delta_old.insert(rd);
    Delta_old.insert(dt);

    int pb = 0;
    for (i = 0; i < 4; i++) {
      for (j = 0; j < 4; j++)
        if (fabs(Delta[i][j] - Delta_old[i][j]) > 1.e-5)
          pb = 1;
    }
    if (pb == 1) {
      printf("pb vpHomogeneousMatrix::expMap\n");
      std::cout << " Delta : " << std::endl << Delta << std::endl;
      std::cout << " Delta_old : " << std::endl << Delta_old << std::endl;
    }
    // end of the test
  }

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

  M.extract(Rd);
  u.buildFrom(Rd);
  for (i = 0; i < 3; i++)
    v[3 + i] = u[i];

  theta = sqrt(u[0] * u[0] + u[1] * u[1] + u[2] * u[2]);
  si = sin(theta);
  co = cos(theta);
  sinc = vpMath::sinc(si, theta);
  mcosc = vpMath::mcosc(co, theta);
  msinc = vpMath::msinc(si, theta);

  // a below is not a pure rotation matrix, even if not so far from
  // the Rodrigues formula : sinc I + (1-sinc)/t^2 VV^T + (1-cos)/t^2 [V]_X
  // with V = t.U

  a[0][0] = sinc + u[0] * u[0] * msinc;
  a[0][1] = u[0] * u[1] * msinc - u[2] * mcosc;
  a[0][2] = u[0] * u[2] * msinc + u[1] * mcosc;

  a[1][0] = u[0] * u[1] * msinc + u[2] * mcosc;
  a[1][1] = sinc + u[1] * u[1] * msinc;
  a[1][2] = u[1] * u[2] * msinc - u[0] * mcosc;

  a[2][0] = u[0] * u[2] * msinc - u[1] * mcosc;
  a[2][1] = u[1] * u[2] * msinc + u[0] * mcosc;
  a[2][2] = sinc + u[2] * u[2] * msinc;

  det = a[0][0] * a[1][1] * a[2][2] + a[1][0] * a[2][1] * a[0][2] + a[0][1] * a[1][2] * a[2][0] -
        a[2][0] * a[1][1] * a[0][2] - a[1][0] * a[0][1] * a[2][2] - a[0][0] * a[2][1] * a[1][2];

  if (fabs(det) > 1.e-5) {
    v[0] = (M[0][3] * a[1][1] * a[2][2] + M[1][3] * a[2][1] * a[0][2] + M[2][3] * a[0][1] * a[1][2] -
            M[2][3] * a[1][1] * a[0][2] - M[1][3] * a[0][1] * a[2][2] - M[0][3] * a[2][1] * a[1][2]) /
           det;
    v[1] = (a[0][0] * M[1][3] * a[2][2] + a[1][0] * M[2][3] * a[0][2] + M[0][3] * a[1][2] * a[2][0] -
            a[2][0] * M[1][3] * a[0][2] - a[1][0] * M[0][3] * a[2][2] - a[0][0] * M[2][3] * a[1][2]) /
           det;
    v[2] = (a[0][0] * a[1][1] * M[2][3] + a[1][0] * a[2][1] * M[0][3] + a[0][1] * M[1][3] * a[2][0] -
            a[2][0] * a[1][1] * M[0][3] - a[1][0] * a[0][1] * M[2][3] - a[0][0] * a[2][1] * M[1][3]) /
           det;
  } else {
    v[0] = M[0][3];
    v[1] = M[1][3];
    v[2] = M[2][3];
  }

  // Apply the sampling time to the computed velocity
  v /= delta_t;

  return (v);
}
