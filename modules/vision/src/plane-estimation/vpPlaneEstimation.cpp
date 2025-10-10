/*
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2025 by Inria. All rights reserved.
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
 * Class for Plane Estimation.
 */

#include <visp3/vision/vpPlaneEstimation.h>

// Check if std:c++17 or higher
#if ((__cplusplus >= 201703L) || (defined(_MSVC_LANG) && (_MSVC_LANG >= 201703L)))

// OpenMP
#ifdef VISP_HAVE_OPENMP
#include <omp.h>
#endif

// Core
#include <visp3/core/vpMeterPixelConversion.h>
#include <visp3/core/vpPixelMeterConversion.h>
#include <visp3/core/vpRobust.h>

BEGIN_VISP_NAMESPACE

#ifndef DOXYGEN_SHOULD_SKIP_THIS
// Local helpers
namespace
{

constexpr auto PlaneSvdMaxError { 1e-4 };
constexpr auto PlaneSvdMaxIter { 10 };

template <class T> T &make_ref(T &&x) { return x; }

/*!
 * Estimate plane equation based on Singular Value Decomposition (SVD).
 *
 * \param[in] point_cloud : Point cloud which represents the plane.
 * \param[out] weights : Plane estimation weights (optional).
 * \return Estimated plane.
 */
vpPlane estimatePlaneEquationSVD(const std::vector<double> &point_cloud, vpColVector &weights = make_ref(vpColVector {}))
{
  // Local helpers
#ifdef VISP_HAVE_OPENMP
  auto num_procs = omp_get_num_procs();
  num_procs = num_procs > 2 ? num_procs - 2 : num_procs;
  omp_set_num_threads(num_procs);
#endif

  auto compute_centroid = [=](const std::vector<double> &point_cloud, const vpColVector &weights) {
    double cent_x { 0. }, cent_y { 0. }, cent_z { 0. }, total_w { 0. };

    int i = 0;
#ifdef VISP_HAVE_OPENMP
#pragma omp parallel for num_threads(num_procs) reduction(+ : total_w, cent_x, cent_y, cent_z)
#endif
    for (i = 0; i < static_cast<int>(weights.size()); i++) {
      const auto pt_cloud_start_idx = 3 * i;

      cent_x += weights[i] * point_cloud[pt_cloud_start_idx + 0];
      cent_y += weights[i] * point_cloud[pt_cloud_start_idx + 1];
      cent_z += weights[i] * point_cloud[pt_cloud_start_idx + 2];

      total_w += weights[i];
    }

    return std::make_tuple(vpColVector { cent_x, cent_y, cent_z }, total_w);
    };

    //
  auto prev_error = 1e3;
  auto error = prev_error - 1;
  const unsigned int nPoints = static_cast<unsigned int>(point_cloud.size() / 3);

  vpColVector residues(nPoints);
  weights = vpColVector(nPoints, 1.0);
  vpColVector normal;
  vpMatrix M(nPoints, 3);
  vpRobust tukey;
  tukey.setMinMedianAbsoluteDeviation(1e-4);

  for (auto iter = 0u; iter < PlaneSvdMaxIter && std::fabs(error - prev_error) > PlaneSvdMaxError; iter++) {
    if (iter != 0) {
      tukey.MEstimator(vpRobust::TUKEY, residues, weights);
    }

    // Compute centroid
#if ((__cplusplus > 201703L) || (defined(_MSVC_LANG) && (_MSVC_LANG > 201703L)))
    auto [centroid, total_w] = compute_centroid(point_cloud, weights);
#else
    // C++17 structured binding are not fully supported by clang 13.0 on macOS
    // See
    // https://stackoverflow.com/questions/46114214/lambda-implicit-capture-fails-with-variable-declared-from-structured-binding
    vpColVector centroid;
    double total_w;
    std::tie(centroid, total_w) = compute_centroid(point_cloud, weights);
#endif

    centroid /= total_w;

    // Minimization
    int i = 0;
#ifdef VISP_HAVE_OPENMP
#pragma omp parallel for num_threads(num_procs)
#endif
    for (i = 0; i < static_cast<int>(nPoints); i++) {
      const auto pt_cloud_start_idx = 3 * i;

      M[i][0] = weights[i] * (point_cloud[pt_cloud_start_idx + 0] - centroid[0]);
      M[i][1] = weights[i] * (point_cloud[pt_cloud_start_idx + 1] - centroid[1]);
      M[i][2] = weights[i] * (point_cloud[pt_cloud_start_idx + 2] - centroid[2]);
    }

    vpColVector W {};
    vpMatrix V {};
    auto J = M.t() * M;
    J.svd(W, V);

    auto smallestSv = W[0];
    auto indexSmallestSv = 0u;
    for (auto i = 1u; i < W.size(); i++) {
      if (W[i] < smallestSv) {
        smallestSv = W[i];
        indexSmallestSv = i;
      }
    }

    normal = V.getCol(indexSmallestSv);

    // Compute plane equation
    const auto A = normal[0], B = normal[1], C = normal[2];
    const auto D = -(A * centroid[0] + B * centroid[1] + C * centroid[2]);

    // Compute error points to estimated plane
    prev_error = error;
    error = 0.;
    const auto smth = std::hypot(A, B, C);

#ifdef VISP_HAVE_OPENMP
#pragma omp parallel for num_threads(num_procs) reduction(+ : error)
#endif
    for (i = 0; i < static_cast<int>(nPoints); i++) {
      const auto pt_cloud_start_idx = 3 * i;

      residues[i] = std::fabs(A * point_cloud[pt_cloud_start_idx + 0] + B * point_cloud[pt_cloud_start_idx + 1] +
                              C * point_cloud[pt_cloud_start_idx + 2] + D) /
        smth;

      error += weights[i] * residues[i];
    }

    error /= total_w;
  }

  // Update final weights
  tukey.MEstimator(vpRobust::TUKEY, residues, weights);

  // Update final centroid
  auto [centroid, total_w] = compute_centroid(point_cloud, weights);
  centroid /= total_w;

  // Compute final plane equation
  const auto A = normal[0], B = normal[1], C = normal[2];
  const auto D = -(A * centroid[0] + B * centroid[1] + C * centroid[2]);

  // Return final plane equation
  return { A, B, C, D };
}

/**
 * \brief Get the Helpers object
 *
 * \param[in] roi
 * \param[in] height
 * \param[in] width
 * \param[in] MaxSubSampFactorToEstimatePlane
 * \param[in] avg_nb_of_pts_to_estimate
 * \param[in] isInside Local helper: Reduce computation (roi.isInside)
 * \param[in] subsample_factor
 */
void getHelpers(const vpPolygon &roi, const unsigned int &height, const unsigned int &width,
                 const unsigned int &MaxSubSampFactorToEstimatePlane, const unsigned int &avg_nb_of_pts_to_estimate,
                 std::function<bool(const vpImagePoint &, const vpPolygon &)> &isInside,
                 unsigned int &subsample_factor,
                 int &roi_top, int &roi_bottom, int &roi_left, int &roi_right
)
{
  // If the img is crossed by the ROI, vpPolygon::isInside has to be used
  {
    // If at least one ROI corner is inside the img bound
    const vpRect img_bound { vpImagePoint(0, 0), static_cast<double>(width),
                           static_cast<double>(height) };
    for (const auto &roi_corner : roi.getCorners()) {
      if (img_bound.isInside(roi_corner)) {
        isInside = [](const vpImagePoint &ip, const vpPolygon &roi) { return roi.isInside(ip); };
        break;
      }
    }

    // If at least one img corner is outside the ROI
    // clang-format off
    if (!roi.isInside(img_bound.getTopLeft()) ||
         !roi.isInside(img_bound.getTopRight()) ||
         !roi.isInside(img_bound.getBottomLeft()) ||
         !roi.isInside(img_bound.getBottomRight()))
    // clang-format on
    {
      isInside = [](const vpImagePoint &ip, const vpPolygon &roi) { return roi.isInside(ip); };
    }
  }

  // Limit research area
  const auto roi_bb = roi.getBoundingBox();
  roi_top = static_cast<int>(std::max<double>(0., roi_bb.getTop()));
  roi_bottom = static_cast<int>(std::min<double>(static_cast<double>(height), roi_bb.getBottom()));
  roi_left = static_cast<int>(std::max<double>(0., roi_bb.getLeft()));
  roi_right = static_cast<int>(std::min<double>(static_cast<double>(width), roi_bb.getRight()));

  // Reduce computation time by using subsample factor
  subsample_factor =
    static_cast<int>(sqrt(((roi_right - roi_left) * (roi_bottom - roi_top)) / avg_nb_of_pts_to_estimate));
  subsample_factor = vpMath::clamp(subsample_factor, 1u, MaxSubSampFactorToEstimatePlane);
}

std::optional<vpPlane> estimatePlaneEquation(const std::vector<double> &pt_cloud, const unsigned int &MinPointNbToEstimatePlane,
                                  const unsigned int &height, const unsigned int &width, const vpCameraParameters &depth_intrinsics,
                                 std::optional<std::reference_wrapper<vpImage<vpRGBa> > > heat_map)
{
  if (pt_cloud.size() < MinPointNbToEstimatePlane) {
    return std::nullopt;
  }

  // Display heatmap
  if (heat_map) {
    vpColVector weights {};
    const auto plane = estimatePlaneEquationSVD(pt_cloud, weights);

    heat_map->get() = vpImage<vpRGBa> { height, width, vpColor::black };

    for (auto i = 0u; i < weights.size(); i++) {
      const auto X { pt_cloud[3 * i + 0] }, Y { pt_cloud[3 * i + 1] }, Z { pt_cloud[3 * i + 2] };

      vpImagePoint ip {};
      vpMeterPixelConversion::convertPoint(depth_intrinsics, X / Z, Y / Z, ip);

      const int b = static_cast<int>(std::max<double>(0., 255 * (1 - 2 * weights[i])));
      const int r = static_cast<int>(std::max<double>(0., 255 * (2 * weights[i] - 1)));
      const int g = 255 - b - r;

      heat_map->get()[static_cast<int>(ip.get_i())][static_cast<int>(ip.get_j())] = vpColor(r, g, b);
    }
    return plane;
  }
  else {
    return estimatePlaneEquationSVD(pt_cloud);
  }
}
} // namespace

#endif // DOXYGEN_SHOULD_SKIP_THIS
std::optional<vpPlane>
vpPlaneEstimation::estimatePlane(const vpImage<uint16_t> &I_depth_raw, double depth_scale,
                                 const vpCameraParameters &depth_intrinsics, const vpPolygon &roi,
                                 const unsigned int avg_nb_of_pts_to_estimate,
                                 std::optional<std::reference_wrapper<vpImage<vpRGBa> > > heat_map)
{
  return estimatePlane(I_depth_raw, depth_scale, depth_intrinsics, roi, std::nullopt, avg_nb_of_pts_to_estimate, heat_map);
}

std::optional<vpPlane>
vpPlaneEstimation::estimatePlane(const vpImage<uint16_t> &I_depth_raw, double depth_scale,
                                 const vpCameraParameters &depth_intrinsics, const vpPolygon &roi,
                                 const std::optional<vpImage<bool>> &mask,
                                 const unsigned int avg_nb_of_pts_to_estimate,
                                 std::optional<std::reference_wrapper<vpImage<vpRGBa> > > heat_map)
{
  // Local helper: Reduce computation (roi.isInside)
  // Default: the img is totally included in the ROI
  std::function<bool(const vpImagePoint &, const vpPolygon &)> isInside = [](const vpImagePoint &, const vpPolygon &) { return true; };

  // Local helper: check if the considered point must be considered according to the mask
  // Default: all the points must be considered
  std::function<bool(const vpImagePoint &)> isValid = [](const vpImagePoint &) { return true; };

  if (mask) {
    isValid = [&mask](const vpImagePoint &ip) { return (*mask)[static_cast<unsigned int>(ip.get_i())][static_cast<unsigned int>(ip.get_j())]; };
  }

  unsigned int subsample_factor = 0;
  int roi_top = 0, roi_bottom = 0, roi_left = 0, roi_right = 0;
  getHelpers(roi, I_depth_raw.getHeight(), I_depth_raw.getWidth(),
              MaxSubSampFactorToEstimatePlane, avg_nb_of_pts_to_estimate,
              isInside, subsample_factor, roi_top, roi_bottom, roi_left, roi_right);

  // Create the point cloud which will be used for plane estimation
  std::vector<double> pt_cloud {};

#if defined(VISP_HAVE_OPENMP) && !(_WIN32)
  auto num_procs = omp_get_num_procs();
  num_procs = num_procs > 2 ? num_procs - 2 : num_procs;
  omp_set_num_threads(num_procs);
// The following OpenMP 4.0 directive is not supported by Visual C++ compiler that allows only OpenMP 2.0 support
// https://docs.microsoft.com/en-us/cpp/parallel/openmp/openmp-in-visual-cpp?redirectedfrom=MSDN&view=msvc-170
#pragma omp declare reduction (merge : std::vector<double> : omp_out.insert( end( omp_out ), std::make_move_iterator( begin( omp_in ) ), std::make_move_iterator( end( omp_in ) ) ))
#pragma omp parallel for num_threads(num_procs) collapse(2) reduction(merge : pt_cloud)
#endif
  for (int i = roi_top; i < roi_bottom; i = i + subsample_factor) {
    for (int j = roi_left; j < roi_right; j = j + subsample_factor) {
      const auto pixel = vpImagePoint { static_cast<double>(i), static_cast<double>(j) };
      if ((I_depth_raw[i][j] != 0) && isInside(pixel, roi) && isValid(pixel)) {
        double x { 0. }, y { 0. };
        vpPixelMeterConversion::convertPoint(depth_intrinsics, pixel, x, y);
        const double Z = I_depth_raw[i][j] * depth_scale;

        pt_cloud.push_back(x * Z);
        pt_cloud.push_back(y * Z);
        pt_cloud.push_back(Z);
      }
    }
  }

  return estimatePlaneEquation(pt_cloud, MinPointNbToEstimatePlane,
                               I_depth_raw.getHeight(), I_depth_raw.getWidth(),
                               depth_intrinsics, heat_map);
}

std::optional<vpPlane> vpPlaneEstimation::estimatePlane(const vpImage<float> &I_depth,
    const vpCameraParameters &depth_intrinsics, const vpPolygon &roi, const std::optional<vpImage<bool>> &mask,
    const unsigned int &avg_nb_of_pts_to_estimate,
    std::optional<std::reference_wrapper<vpImage<vpRGBa> > > heat_map)
{
// Local helper: Reduce computation (roi.isInside)
  // Default: the img is totally included in the ROI
  std::function<bool(const vpImagePoint &, const vpPolygon &)> isInside = [](const vpImagePoint &, const vpPolygon &) { return true; };

  // Local helper: check if the considered point must be considered according to the mask
  // Default: all the points must be considered
  std::function<bool(const vpImagePoint &)> isValid = [](const vpImagePoint &) { return true; };

  if (mask) {
    isValid = [&mask](const vpImagePoint &ip) { return (*mask)[static_cast<unsigned int>(ip.get_i())][static_cast<unsigned int>(ip.get_j())]; };
  }

  unsigned int subsample_factor = 0;
  int roi_top = 0, roi_bottom = 0, roi_left = 0, roi_right = 0;
  getHelpers(roi, I_depth.getHeight(), I_depth.getWidth(),
              MaxSubSampFactorToEstimatePlane, avg_nb_of_pts_to_estimate,
              isInside, subsample_factor, roi_top, roi_bottom, roi_left, roi_right);

  // Create the point cloud which will be used for plane estimation
  std::vector<double> pt_cloud {};

#if defined(VISP_HAVE_OPENMP) && !(_WIN32)
  auto num_procs = omp_get_num_procs();
  num_procs = num_procs > 2 ? num_procs - 2 : num_procs;
  omp_set_num_threads(num_procs);
// The following OpenMP 4.0 directive is not supported by Visual C++ compiler that allows only OpenMP 2.0 support
// https://docs.microsoft.com/en-us/cpp/parallel/openmp/openmp-in-visual-cpp?redirectedfrom=MSDN&view=msvc-170
#pragma omp declare reduction (merge : std::vector<double> : omp_out.insert( end( omp_out ), std::make_move_iterator( begin( omp_in ) ), std::make_move_iterator( end( omp_in ) ) ))
#pragma omp parallel for num_threads(num_procs) collapse(2) reduction(merge : pt_cloud)
#endif
  for (int i = roi_top; i < roi_bottom; i = i + subsample_factor) {
    for (int j = roi_left; j < roi_right; j = j + subsample_factor) {
      const auto pixel = vpImagePoint { static_cast<double>(i), static_cast<double>(j) };
      if ((I_depth[i][j] != 0) && isInside(pixel, roi) && isValid(pixel)) {
        double x { 0. }, y { 0. };
        vpPixelMeterConversion::convertPoint(depth_intrinsics, pixel, x, y);
        const double Z = I_depth[i][j];

        pt_cloud.push_back(x * Z);
        pt_cloud.push_back(y * Z);
        pt_cloud.push_back(Z);
      }
    }
  }

  return estimatePlaneEquation(pt_cloud, MinPointNbToEstimatePlane,
                               I_depth.getHeight(), I_depth.getWidth(),
                               depth_intrinsics, heat_map);
}
END_VISP_NAMESPACE
#endif
