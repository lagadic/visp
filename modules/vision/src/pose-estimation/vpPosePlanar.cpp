/****************************************************************************
 *
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2022 by Inria. All rights reserved.
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
 * Pose computation from planar object and N (>3) points.
 *
 * Authors:
 * Julien Dufour
 *
 *****************************************************************************/

#include <visp3/vision/vpPose.h>

#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_17) &&                                                                     \
    (!defined(_MSC_VER) || ((VISP_CXX_STANDARD >= VISP_CXX_STANDARD_17) && (_MSC_VER >= 1911)))

// Internal
#include <visp3/core/vpExponentialMap.h>
#include <visp3/core/vpPixelMeterConversion.h>

namespace
{
/*!
 * Check if the pose transformation matrix doesn't have a value NaN.
 * \param[in] cMo : Input pose
 * \return true when no NaN found, false otherwise.
 */
bool validPose(const vpHomogeneousMatrix &cMo)
{
  for (auto i = 0u; i < cMo.size(); i++) {
    if (vpMath::isNaN(cMo.data[i])) {
      return false;
    }
  }
  return true;
}

/*!
 * Compute Z value of an point located on the plane.
 * \param[in] plane : Plane to consider
 * \param[in] x, y : Coordinates of a point in the image plane.
 * \return Z in [m].
 */
double computeZFromPlane(const vpPlane &plane, double x, double y)
{
  return -plane.getD() / (plane.getA() * x + plane.getB() * y + plane.getC());
}

/*!
 * Compute the transformation between two point clouds.
 * \param[in] p : First point cloud.
 * \param[in] q : Second point cloud.
 * \return The homogeneous transformation \f${^p}{\bf M}_q\f$.
 */
vpHomogeneousMatrix compute3d3dTransformation(const std::vector<vpPoint> &p, const std::vector<vpPoint> &q)
{
  const double N = p.size();

  vpColVector p_bar(3, 0.0);
  vpColVector q_bar(3, 0.0);
  for (auto i = 0u; i < p.size(); i++) {
    for (auto j = 0u; j < 3; j++) {
      p_bar[j] += p.at(i).oP[j];
      q_bar[j] += q.at(i).oP[j];
    }
  }

  for (auto j = 0u; j < 3; j++) {
    p_bar[j] /= N;
    q_bar[j] /= N;
  }

  vpMatrix pc(p.size(), 3);
  vpMatrix qc(q.size(), 3);

  for (auto i = 0u; i < p.size(); i++) {
    for (auto j = 0u; j < 3; j++) {
      pc[i][j] = p.at(i).oP[j] - p_bar[j];
      qc[i][j] = q.at(i).oP[j] - q_bar[j];
    }
  }

  const auto pct_qc = pc.t() * qc;
  vpMatrix U = pct_qc, V;
  vpColVector W;
  U.svd(W, V);

  auto Vt = V.t();
  auto R = U * Vt;
  if (R.det() < 0) {
    Vt[2][0] *= -1;
    Vt[2][1] *= -1;
    Vt[2][2] *= -1;

    R = U * Vt;
  }

  const auto t = p_bar - R * q_bar;

  return {vpTranslationVector(t), vpRotationMatrix(R)};
}

std::optional<vpHomogeneousMatrix> computePoseVVS(std::vector<vpPoint> points, vpHomogeneousMatrix cMo)
{
  auto residu_1{1e8}, r{1e8 - 1};
  const auto lambda{0.9}, vvsEpsilon{1e-8};
  const unsigned int vvsIterMax{200};

  const unsigned int nb = points.size();
  vpMatrix L(3 * nb, 6);
  vpColVector err(3 * nb);
  vpColVector sd(3 * nb), s(3 * nb);

  // create sd
  for (auto i = 0u; i < points.size(); i++) {
    sd[3 * i] = points[i].get_x();
    sd[3 * i + 1] = points[i].get_y();
    sd[3 * i + 2] = points[i].get_Z();
  }

  auto cMoPrev = cMo;
  auto iter = 0u;
  while (std::fabs(residu_1 - r) > vvsEpsilon) {
    residu_1 = r;

    // Compute the interaction matrix and the error
    for (auto i = 0u; i < points.size(); i++) {
      // forward projection of the 3D model for a given pose
      // change frame coordinates
      // perspective projection
      points.at(i).track(cMo);

      const auto x = s[3 * i] = points.at(i).get_x();
      const auto y = s[3 * i + 1] = points.at(i).get_y();
      const auto Z = s[3 * i + 2] = points.at(i).get_Z();
      L[3 * i][0] = -1 / Z;
      L[3 * i][1] = 0;
      L[3 * i][2] = x / Z;
      L[3 * i][3] = x * y;
      L[3 * i][4] = -(1 + vpMath::sqr(x));
      L[3 * i][5] = y;

      L[3 * i + 1][0] = 0;
      L[3 * i + 1][1] = -1 / Z;
      L[3 * i + 1][2] = y / Z;
      L[3 * i + 1][3] = 1 + vpMath::sqr(y);
      L[3 * i + 1][4] = -x * y;
      L[3 * i + 1][5] = -x;

      L[3 * i + 2][0] = 0;
      L[3 * i + 2][1] = 0;
      L[3 * i + 2][2] = -1;
      L[3 * i + 2][3] = -y * Z;
      L[3 * i + 2][4] = x * Z;
      L[3 * i + 2][5] = -0;
    }
    err = s - sd;

    // compute the residual
    r = err.sumSquare();

    // compute the pseudo inverse of the interaction matrix
    vpMatrix Lp;
    L.pseudoInverse(Lp, 1e-16);

    // compute the VVS control law
    const auto v = -lambda * Lp * err;

    // update the pose
    cMoPrev = cMo;
    cMo = vpExponentialMap::direct(v).inverse() * cMo;

    if (iter++ > vvsIterMax) {
      return std::nullopt;
    }
  }
  return cMo;
}

} // namespace

/*!
 * Compute the pose of a planar object from corresponding 2D-3D point coordinates and plane equation.
 * Here at least 3 points are required.
 *
 * \param[in] plane_in_camera_frame : Plane in camera frame.
 * \param[in] pts : Object points.
 * \param[in] ips : Points in the image.
 * \param[in] camera_intrinsics : Camera parameters.
 * \param[in] cMo_init : Camera to object frame transformation used as initialization. When set to `std::nullopt`,
 * this transformation is computed internally.
 * \param[in] enable_vvs : When true, refine estimated pose using a virtual visual servoing scheme.
 * \return Homogeneous matrix \f${^c}{\bf M}_o\f$ between camera frame and object frame when estimation succeed,
 * nullopt otherwise.
 */
template <typename DataId>
std::optional<vpHomogeneousMatrix>
vpPose::computePlanarObjectPoseFrom3Points(const vpPlane &plane_in_camera_frame, const std::map<DataId, vpPoint> &pts,
                                           const std::map<DataId, vpImagePoint> &ips,
                                           const vpCameraParameters &camera_intrinsics,
                                           std::optional<vpHomogeneousMatrix> cMo_init, bool enable_vvs)
{
  if (cMo_init && not enable_vvs) {
    throw(vpException(
        vpException::fatalError,
        "It doesn't make sense to use an initialized pose without enabling VVS to compute the pose from 4 points"));
  }

  // Check if detection and model fit
  for ([[maybe_unused]] const auto &[ip_id, _] : ips) {
    if (pts.find(ip_id) == end(pts)) {
      throw(vpException(vpException::fatalError,
                        "Cannot compute pose with points and image points which do not have the same IDs"));
    }
  }

  std::vector<vpPoint> P{}, Q{};
  for (auto [pt_id, pt] : pts) {
    if (ips.find(pt_id) != end(ips)) {
      double x = 0, y = 0;
      vpPixelMeterConversion::convertPoint(camera_intrinsics, ips.at(pt_id), x, y);
      const auto Z = computeZFromPlane(plane_in_camera_frame, x, y);

      pt.set_x(x);
      pt.set_y(y);
      pt.set_Z(Z);

      Q.push_back(pt);
      P.emplace_back(x * Z, y * Z, Z);
    }
  }

  if (Q.size() < 3) {
    return std::nullopt;
  }

  auto cMo = cMo_init.value_or(compute3d3dTransformation(P, Q));
  if (not validPose(cMo)) {
    return std::nullopt;
  }

  return enable_vvs ? computePoseVVS(Q, cMo).value_or(cMo) : cMo;
}

template std::optional<vpHomogeneousMatrix> vpPose::computePlanarObjectPoseFrom3Points<unsigned long int>(
    const vpPlane &, const std::map<unsigned long int, vpPoint> &, const std::map<unsigned long int, vpImagePoint> &,
    const vpCameraParameters &, std::optional<vpHomogeneousMatrix>, bool);

#endif
