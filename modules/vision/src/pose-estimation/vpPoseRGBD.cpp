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
 * Pose computation from RGBD.
 */

#include <visp3/core/vpPixelMeterConversion.h>
#include <visp3/core/vpPlane.h>
#include <visp3/core/vpPolygon.h>
#include <visp3/core/vpRobust.h>
#include <visp3/vision/vpPose.h>

BEGIN_VISP_NAMESPACE

// See also vpPlaneEstimation.cpp that implements the same functionaly in c++17
void estimatePlaneEquationSVD(const std::vector<double> &point_cloud_face, vpPlane &plane_equation_estimated,
                              vpColVector &centroid, double &normalized_weights)
{
  const unsigned int max_iter = 10;
  double prev_error = 1e3;
  double error = 1e3 - 1;
  const unsigned int size3DPt = 3;
  unsigned int nPoints = static_cast<unsigned int>(point_cloud_face.size() / size3DPt);
  const unsigned int idX = 0;
  const unsigned int idY = 1;
  const unsigned int idZ = 2;

  vpColVector weights(nPoints, 1.0);
  vpColVector residues(nPoints);
  vpMatrix M(nPoints, size3DPt);
  vpRobust tukey;
  tukey.setMinMedianAbsoluteDeviation(1e-4);
  vpColVector normal;

  double fabs_error_m_prev_error = std::fabs(error - prev_error);
  unsigned int iter = 0;
  while ((iter < max_iter) && (fabs_error_m_prev_error > 1e-6)) {
    if (iter != 0) {
      tukey.MEstimator(vpRobust::TUKEY, residues, weights);
    }

    // Compute centroid
    double centroid_x = 0.0, centroid_y = 0.0, centroid_z = 0.0;
    double total_w = 0.0;

    for (unsigned int i = 0; i < nPoints; ++i) {
      centroid_x += weights[i] * point_cloud_face[(size3DPt * i) + idX];
      centroid_y += weights[i] * point_cloud_face[(size3DPt * i) + idY];
      centroid_z += weights[i] * point_cloud_face[(size3DPt * i) + idZ];
      total_w += weights[i];
    }

    centroid_x /= total_w;
    centroid_y /= total_w;
    centroid_z /= total_w;

    // Minimization
    for (unsigned int i = 0; i < nPoints; ++i) {
      M[static_cast<unsigned int>(i)][idX] = weights[i] * (point_cloud_face[(size3DPt * i) + idX] - centroid_x);
      M[static_cast<unsigned int>(i)][idY] = weights[i] * (point_cloud_face[(size3DPt * i) + idY] - centroid_y);
      M[static_cast<unsigned int>(i)][idZ] = weights[i] * (point_cloud_face[(size3DPt * i) + idZ] - centroid_z);
    }

    vpColVector W;
    vpMatrix V;
    vpMatrix J = M.t() * M;
    J.svd(W, V);

    double smallestSv = W[0];
    unsigned int indexSmallestSv = 0;
    unsigned int w_size = W.size();
    for (unsigned int i = 1; i < w_size; ++i) {
      if (W[i] < smallestSv) {
        smallestSv = W[i];
        indexSmallestSv = i;
      }
    }

    normal = V.getCol(indexSmallestSv);

    // Compute plane equation
    double A = normal[idX];
    double B = normal[idY];
    double C = normal[idZ];
    double D = -((A * centroid_x) + (B * centroid_y) + (C * centroid_z));

    // Compute error points to estimated plane
    prev_error = error;
    error = 0.0;
    for (unsigned int i = 0; i < nPoints; ++i) {
      residues[i] = std::fabs((A * point_cloud_face[size3DPt * i]) + (B * point_cloud_face[(size3DPt * i) + idY]) +
                              (C * point_cloud_face[(size3DPt * i) + idZ]) + D) /
        sqrt((A * A) + (B * B) + (C * C));
      error += weights[i] * residues[i];
    }
    error /= total_w;
    // evaluate one of the end conditions of the for
    fabs_error_m_prev_error = std::fabs(error - prev_error);

    ++iter;
  }

  // Update final weights
  tukey.MEstimator(vpRobust::TUKEY, residues, weights);

  // Update final centroid
  centroid.resize(size3DPt, false);
  double total_w = 0.0;

  for (unsigned int i = 0; i < nPoints; ++i) {
    centroid[idX] += weights[i] * point_cloud_face[size3DPt * i];
    centroid[idY] += weights[i] * point_cloud_face[(size3DPt * i) + idY];
    centroid[idZ] += weights[i] * point_cloud_face[(size3DPt * i) + idZ];
    total_w += weights[i];
  }

  centroid[idX] /= total_w;
  centroid[idY] /= total_w;
  centroid[idZ] /= total_w;

  // Compute final plane equation
  double A = normal[0], B = normal[1], C = normal[idZ];
  double D = -((A * centroid[0]) + (B * centroid[1]) + (C * centroid[idZ]));

  // Update final plane equation
  plane_equation_estimated.setABCD(A, B, C, D);

  normalized_weights = total_w / nPoints;
}

bool vpPose::computePlanarObjectPoseFromRGBD(const vpImage<float> &depthMap, const std::vector<vpImagePoint> &corners,
                                             const vpCameraParameters &colorIntrinsics,
                                             const std::vector<vpPoint> &point3d, vpHomogeneousMatrix &cMo,
                                             double *confidence_index)
{
  if (corners.size() != point3d.size()) {
    throw(vpException(vpException::fatalError,
                      "Cannot compute pose from RGBD, 3D (%d) and 2D (%d) data doesn't have the same size",
                      point3d.size(), corners.size()));
  }
  std::vector<vpPoint> pose_points;
  if (confidence_index != nullptr) {
    *confidence_index = 0.0;
  }

  size_t point3d_size = point3d.size();
  for (size_t i = 0; i < point3d_size; ++i) {
    pose_points.push_back(point3d[i]);
  }

  vpPolygon polygon(corners);
  vpRect bb = polygon.getBoundingBox();
  unsigned int top = static_cast<unsigned int>(std::max<int>(0, static_cast<int>(bb.getTop())));
  unsigned int bottom =
    static_cast<unsigned int>(std::min<int>(static_cast<int>(depthMap.getHeight()) - 1, static_cast<int>(bb.getBottom())));
  unsigned int left = static_cast<unsigned int>(std::max<int>(0, static_cast<int>(bb.getLeft())));
  unsigned int right =
    static_cast<unsigned int>(std::min<int>(static_cast<int>(depthMap.getWidth()) - 1, static_cast<int>(bb.getRight())));

  std::vector<double> points_3d;
  points_3d.reserve((bottom - top) * (right - left));
  for (unsigned int idx_i = top; idx_i < bottom; ++idx_i) {
    for (unsigned int idx_j = left; idx_j < right; ++idx_j) {
      vpImagePoint imPt(idx_i, idx_j);
      if ((depthMap[idx_i][idx_j] > 0) && (polygon.isInside(imPt))) {
        double x = 0, y = 0;
        vpPixelMeterConversion::convertPoint(colorIntrinsics, imPt.get_u(), imPt.get_v(), x, y);
        double Z = depthMap[idx_i][idx_j];
        points_3d.push_back(x * Z);
        points_3d.push_back(y * Z);
        points_3d.push_back(Z);
      }
    }
  }

  unsigned int nb_points_3d = static_cast<unsigned int>(points_3d.size() / 3);

  if (nb_points_3d > 4) {
    std::vector<vpPoint> p, q;

    // Plane equation
    vpPlane plane_equation;
    vpColVector centroid;
    double normalized_weights = 0;
    estimatePlaneEquationSVD(points_3d, plane_equation, centroid, normalized_weights);

    size_t corners_size = corners.size();
    for (size_t j = 0; j < corners_size; ++j) {
      const vpImagePoint &imPt = corners[j];
      double x = 0, y = 0;
      vpPixelMeterConversion::convertPoint(colorIntrinsics, imPt.get_u(), imPt.get_v(), x, y);
      double Z = plane_equation.computeZ(x, y);
      if (Z < 0) {
        Z = -Z;
      }
      p.push_back(vpPoint(x * Z, y * Z, Z));

      pose_points[j].set_x(x);
      pose_points[j].set_y(y);
    }

    size_t point3d_size = point3d.size();
    for (size_t i = 0; i < point3d_size; ++i) {
      q.push_back(point3d[i]);
    }

    cMo = vpHomogeneousMatrix::compute3d3dTransformation(p, q);

    if (cMo.isValid()) {
      vpPose pose;
      pose.addPoints(pose_points);
      if (pose.computePose(vpPose::VIRTUAL_VS, cMo)) {
        if (confidence_index != nullptr) {
          *confidence_index = std::min<double>(1.0, (normalized_weights * static_cast<double>(nb_points_3d)) / polygon.getArea());
        }
        return true;
      }
    }
  }

  return false;
}

bool vpPose::computePlanarObjectPoseFromRGBD(const vpImage<float> &depthMap,
                                             const std::vector<std::vector<vpImagePoint> > &corners,
                                             const vpCameraParameters &colorIntrinsics,
                                             const std::vector<std::vector<vpPoint> > &point3d,
                                             vpHomogeneousMatrix &cMo, double *confidence_index, bool coplanar_points)
{
  const size_t nb3dPoints = point3d.size();
  const size_t nbCorners = corners.size();
  if (nbCorners != nb3dPoints) {
    throw(vpException(vpException::fatalError,
                      "Cannot compute pose from RGBD, 3D (%d) and 2D (%d) data doesn't have the same size",
                      nb3dPoints, nbCorners));
  }
  std::vector<vpPoint> pose_points;
  if (confidence_index != nullptr) {
    *confidence_index = 0.0;
  }

  for (size_t i = 0; i < nb3dPoints; ++i) {
    std::vector<vpPoint> tagPoint3d = point3d[i];
    size_t tagpoint3d_size = tagPoint3d.size();
    for (size_t j = 0; j < tagpoint3d_size; ++j) {
      pose_points.push_back(tagPoint3d[j]);
    }
  }

  // Total area of the polygon to estimate confidence
  double totalArea = 0.0;

  // If coplanar is true, the tags_points_3d  will be used to compute one plane
  std::vector<double> tag_points_3d;

  // Otherwise the vector of planes will be used to compute each plane for each vector
  std::vector<std::vector<double> > tag_points_3d_nonplanar;
  size_t nb_points_3d_non_planar = 0;

  // Loop through each object, compute 3d point cloud of each
  size_t corners_size = corners.size();
  for (size_t i = 0; i < corners_size; ++i) {
    std::vector<double> points_3d;
    vpPolygon polygon(corners[i]);
    vpRect bb = polygon.getBoundingBox();

    // The area to calculate final confidence index should be total area of the tags
    totalArea += polygon.getArea();

    unsigned int top = static_cast<unsigned int>(std::max<int>(0, static_cast<int>(bb.getTop())));
    unsigned int bottom = static_cast<unsigned int>(
        std::min<int>(static_cast<int>(depthMap.getHeight()) - 1, static_cast<int>(bb.getBottom())));
    unsigned int left = static_cast<unsigned int>(std::max<int>(0, static_cast<int>(bb.getLeft())));
    unsigned int right =
      static_cast<unsigned int>(std::min<int>(static_cast<int>(depthMap.getWidth()) - 1, static_cast<int>(bb.getRight())));

    points_3d.reserve((bottom - top) * (right - left));
    for (unsigned int idx_i = top; idx_i < bottom; ++idx_i) {
      for (unsigned int idx_j = left; idx_j < right; ++idx_j) {
        vpImagePoint imPt(idx_i, idx_j);
        if ((depthMap[idx_i][idx_j] > 0) && (polygon.isInside(imPt))) {
          double x = 0, y = 0;
          vpPixelMeterConversion::convertPoint(colorIntrinsics, imPt.get_u(), imPt.get_v(), x, y);
          double Z = depthMap[idx_i][idx_j];
          points_3d.push_back(x * Z);
          points_3d.push_back(y * Z);
          points_3d.push_back(Z);
        }
      }
    }

    // If coplanar_points is true, feed all 3d points to single vector
    // Otherwise, each vector will hold 3d points for separate planes
    if (coplanar_points) {
      tag_points_3d.insert(tag_points_3d.end(), points_3d.begin(), points_3d.end());
    }
    else {
      tag_points_3d_nonplanar.push_back(points_3d);
      nb_points_3d_non_planar += points_3d.size();
    }
  }

  size_t nb_points_3d = 0;
  const size_t sizePt3D = 3;

  if (coplanar_points) {
    nb_points_3d = tag_points_3d.size() / sizePt3D;
  }
  else {
    nb_points_3d = nb_points_3d_non_planar / sizePt3D;
  }

  const size_t minNbPts = 4;
  if (nb_points_3d > minNbPts) {
    std::vector<vpPoint> p, q;

    // Plane equation
    vpPlane plane_equation;
    vpColVector centroid;
    double normalized_weights = 0;

    if (coplanar_points) {
      // If all objects are coplanar, use points insides tag_points_3d to estimate the plane
      estimatePlaneEquationSVD(tag_points_3d, plane_equation, centroid, normalized_weights);
      int count = 0;
      for (size_t j = 0; j < nbCorners; ++j) {
        std::vector<vpImagePoint> tag_corner = corners[j];
        size_t tag_corner_size = tag_corner.size();
        for (size_t i = 0; i < tag_corner_size; ++i) {
          const vpImagePoint &imPt = tag_corner[i];
          double x = 0, y = 0;
          vpPixelMeterConversion::convertPoint(colorIntrinsics, imPt.get_u(), imPt.get_v(), x, y);
          double Z = plane_equation.computeZ(x, y);
          std::cout << Z;
          if (Z < 0) {
            Z = -Z;
          }
          p.push_back(vpPoint(x * Z, y * Z, Z));
          pose_points[count].set_x(x);
          pose_points[count].set_y(y);
          ++count;
        }
      }
    }
    else {
   // If the tags is not coplanar, estimate the plane for each tags
      size_t count = 0;

      size_t tag_points_3d_nonplanar_size = tag_points_3d_nonplanar.size();
      for (size_t k = 0; k < tag_points_3d_nonplanar_size; ++k) {
        std::vector<double> rec_points_3d = tag_points_3d_nonplanar[k];
        double tag_normalized_weights = 0;
        const size_t minNbPtsForPlaneSVD = 3;
        const size_t minSizeForPlaneSVD = minNbPtsForPlaneSVD * sizePt3D;
        if (rec_points_3d.size() >= minSizeForPlaneSVD) {
          // The array must has at least 3 points for the function estimatePlaneEquationSVD not to crash
          estimatePlaneEquationSVD(rec_points_3d, plane_equation, centroid, tag_normalized_weights);
          normalized_weights += tag_normalized_weights;

          // Get the 2d points of the tag the plane just recomputed
          std::vector<vpImagePoint> tag_corner = corners[k];

          size_t tag_corner_size = tag_corner.size();
          for (size_t i = 0; i < tag_corner_size; ++i) {
            const vpImagePoint &imPt = tag_corner[i];
            double x = 0, y = 0;
            vpPixelMeterConversion::convertPoint(colorIntrinsics, imPt.get_u(), imPt.get_v(), x, y);
            double Z = plane_equation.computeZ(x, y);

            if (Z < 0) {
              Z = -Z;
            }
            p.push_back(vpPoint(x * Z, y * Z, Z));
            pose_points[count].set_x(x);
            pose_points[count].set_y(y);
            ++count;
          }
        }
        else {
       // Sometimes an object may do not have enough points registered due to small size or bad alignment btw depth
       // and rgb. This behavior happens with Orbbec camera while Realsenses was fine. To prevent exception while
       // computePose, skip recomputing the failed estimation tag's (4 point - corners)
          count += corners[k].size();
        }
      }
      normalized_weights = normalized_weights / tag_points_3d_nonplanar.size();
    }

    for (size_t i = 0; i < nb3dPoints; ++i) {
      std::vector<vpPoint> tagPoint3d = point3d[i];
      // Sometimes an object may do not have enough points registered due to small size.
      // The issue happens with Orbbec camera while Realsenses was fine.
      // To prevent wrong estimation or exception (p and q sizes are differents),
      // ignore the recomputer vector (tag_points_3d_nonplanar) when size = 0
      if (coplanar_points) {
        size_t tag_point3d_size = tagPoint3d.size();
        for (size_t j = 0; j < tag_point3d_size; ++j) {
          q.push_back(tagPoint3d[j]);
        }
      }
      else {
        if (tag_points_3d_nonplanar[i].size() > 0) {
          size_t tag_point3d_size = tagPoint3d.size();
          for (size_t j = 0; j < tag_point3d_size; ++j) {
            q.push_back(tagPoint3d[j]);
          }
        }
      }
    }

    // Due to the possibility of q's size might less than p's, check their size should be identical
    if (p.size() == q.size()) {
      cMo = vpHomogeneousMatrix::compute3d3dTransformation(p, q);

      if (cMo.isValid()) {
        vpPose pose;
        pose.addPoints(pose_points);
        if (pose.computePose(vpPose::VIRTUAL_VS, cMo)) {
          if (confidence_index != nullptr) {
            *confidence_index = std::min<double>(1.0, (normalized_weights * static_cast<double>(nb_points_3d)) / totalArea);
          }
          return true;
        }
      }
    }
  }
  return false;
}

END_VISP_NAMESPACE
