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
 */

#ifndef VP_RB_VISUAL_ODOMETRY_UTILS_H
#define VP_RB_VISUAL_ODOMETRY_UTILS_H

#include <visp3/core/vpConfig.h>
#include <vector>

BEGIN_VISP_NAMESPACE

class vpRBFeatureTrackerInput;
class vpMatrix;
class vpHomogeneousMatrix;

struct VISP_EXPORT vpLevenbergMarquardtParameters
{
  vpLevenbergMarquardtParameters() : gain(1.0), muInit(0.01), muIterFactor(0.9), minImprovementFactor(0.0), maxNumIters(10) { }
  double gain;
  double muInit;
  double muIterFactor;
  double minImprovementFactor;
  unsigned int maxNumIters;
};

class VISP_EXPORT vpRBVisualOdometryUtils
{

public:
  /**
   * \brief Compute the indices of the keypoints that belong to either the object (the renderered depth is greater than 0) or the environment
   * Keypoints that are too close to the silhouette edges are discarded.
   * There are two thresholds, one for object candidates and another for environment candidates
   * Points can also be filtered with the probability mask, if it is available and minMaskConfidence is greater than 0
   *
   * \param keypoints the keypoints that should be split between object and environment
   * \param frame Input frame data, containing the renders
   * \param minMaskConfidence The minimum mask confidence value to consider a point has belonging to the object. If 0, it is unused
   * \param minEdgeDistObject Minimum euclidean distance between an object keypoint and the object silhouette's points to consider the keypoint as valid
   * \param minEdgeDistEnv Minimum euclidean distance between an environment keypoint and the object silhouette's points to consider the keypoint as a valid one
   * \return std::pair<std::vector<unsigned int>, std::vector<unsigned int>>
  */
  static std::pair<std::vector<unsigned int>, std::vector<unsigned int>> computeIndicesObjectAndEnvironment(
    const vpMatrix &keypoints, const vpRBFeatureTrackerInput &frame, double minMaskConfidence, double minEdgeDistObject, double minEdgeDistEnv);

  static void levenbergMarquardtKeypoints2D(const vpMatrix &points3d, const vpMatrix &observations, const vpLevenbergMarquardtParameters &parameters, vpHomogeneousMatrix &cTw);
};

END_VISP_NAMESPACE

#endif
