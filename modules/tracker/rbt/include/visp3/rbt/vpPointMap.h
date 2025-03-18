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

#ifndef VP_POINT_MAP_H
#define VP_POINT_MAP_H

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpMatrix.h>
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpCameraParameters.h>
#include <visp3/core/vpMeterPixelConversion.h>
#include <visp3/core/vpPixelMeterConversion.h>

#include <list>

BEGIN_VISP_NAMESPACE

class VISP_EXPORT vpPointMap
{
public:
  vpPointMap(unsigned maxPoints, double minDistNewPoints, double maxDepthErrorVisibility, double outlierThreshold)
  {
    m_maxPoints = maxPoints;
    m_minDistNewPoint = minDistNewPoints;
    m_maxDepthError = maxDepthErrorVisibility;
    m_outlierThreshold = outlierThreshold;
  }
  /**
   * \name Settings
   *
   * @{
  */
  unsigned int getNumMaxPoints() const { return m_maxPoints; }
  void setNumMaxPoints(unsigned int maxNumPoints)
  {
    m_maxPoints = maxNumPoints;
  }

  double getMinDistanceAddNewPoints() const { return m_minDistNewPoint; }
  void setMinDistanceAddNewPoints(double distance) { m_minDistNewPoint = distance; }

  double getMaxDepthErrorVisibilityCriterion() const { return m_maxDepthError; }
  void setMaxDepthErrorVisibilityCriterion(double depthError) { m_maxDepthError = depthError; }

  double getOutlierReprojectionErrorThreshold() const { return m_outlierThreshold; }
  void setOutlierReprojectionErrorThreshold(double thresholdPx) { m_outlierThreshold = thresholdPx; }

  /**
   * @}
  */

  const vpMatrix &getPoints() { return m_X; }
  void setPoints(const vpMatrix &X) { m_X = X; }

  void getPoints(const vpArray2D<int> &indices, vpMatrix &X);

  void project(const vpHomogeneousMatrix &cTw, vpMatrix &cX);
  void project(const vpArray2D<int> &indices, const vpHomogeneousMatrix &cTw, vpMatrix &cX);
  void project(const vpArray2D<int> &indices, const vpHomogeneousMatrix &cTw, vpMatrix &cX, vpMatrix &xs);
  void project(const vpCameraParameters &cam, const vpArray2D<int> &indices, const vpHomogeneousMatrix &cTw, vpMatrix &cX, vpMatrix &xs, vpMatrix &uvs);


  void getVisiblePoints(const unsigned int h, const unsigned int w, const vpMatrix &cX, const vpMatrix &uvs, const vpColVector &expectedZ, std::list<int> &indices);
  void getVisiblePoints(const unsigned int h, const unsigned int w, const vpCameraParameters &cam, const vpHomogeneousMatrix &cTw, const vpImage<float> &depth, std::list<int> &indices);

  void getOutliers(const vpArray2D<int> &originalIndices, const vpMatrix &uvs,
  const vpMatrix &observations, std::list<int> &indices);

  void selectValidNewCandidates(const vpCameraParameters &cam, const vpHomogeneousMatrix &cTw,
   const vpArray2D<int> &originalIndices, const vpMatrix &uvs,
   const vpImage<float> &depth, vpMatrix &oXs, std::list<int> &validCandidateIndices);

  void updatePoints(const vpArray2D<int> &indicesToRemove, const vpMatrix &pointsToAdd, std::list<int> &removedIndices, unsigned int &numAddedPoints);
  void updatePoint(unsigned int index, double X, double Y, double Z)
  {
    m_X[index][0] = X;
    m_X[index][1] = Y;
    m_X[index][2] = Z;
  }

  void computeReprojectionErrorAndJacobian(const vpArray2D<int> &indices, const vpHomogeneousMatrix &cTw, const vpMatrix &observations, vpMatrix &J, vpColVector &e) const
  {
    J.resize(indices.getRows() * 2, 6, false, false);
    e.resize(indices.getRows() * 2, 1, false);

    vpColVector cX(3);
    vpColVector wX(3);
    const vpRotationMatrix cRw = cTw.getRotationMatrix();
    const vpTranslationVector t = cTw.getTranslationVector();
    for (unsigned int i = 0; i < indices.getRows(); ++i) {
      const unsigned int pointIndex = indices[i][0];
      const double *p = m_X[pointIndex];
      wX[0] = p[0]; wX[1] = p[1]; wX[2] = p[2];
      cX = cRw * wX;
      cX += t;
      const double Z = cX[2];
      const double x = cX[0] / Z;
      const double y = cX[1] / Z;

      e[i * 2] = x - observations[i][0];
      e[i * 2 + 1] = y - observations[i][1];

      J[i * 2][0] = -1.0 / Z; J[i * 2][1] = 0.0; J[i * 2][2] = x / Z;
      J[i * 2][3] = x * y; J[i * 2][4] = -(1.0 + x * x); J[i * 2][5] = y;

      J[i * 2 + 1][0] = 0.0; J[i * 2 + 1][1] = -1.0 / Z; J[i * 2 + 1][2] = y / Z;
      J[i * 2 + 1][3] = 1.0 + y * y; J[i * 2 + 1][4] = -(x * y); J[i * 2 + 1][5] = -x;
    }
  }

  void compute3DErrorAndJacobian(const vpArray2D<int> &indices, const vpHomogeneousMatrix &cTw, const vpMatrix &observations, vpMatrix &J, vpColVector &e) const
  {
    J.resize(indices.getRows() * 3, 6, false, false);
    e.resize(indices.getRows() * 3, 1, false);

    vpColVector cX(3);
    vpColVector wX(3);
    const vpRotationMatrix cRw = cTw.getRotationMatrix();
    const vpTranslationVector t = cTw.getTranslationVector();
    for (unsigned int i = 0; i < indices.getRows(); ++i) {
      const unsigned int pointIndex = indices[i][0];
      const double *p = m_X[pointIndex];
      wX[0] = p[0]; wX[1] = p[1]; wX[2] = p[2];
      cX = cRw * wX;
      cX += t;

      const double X = cX[0], Y = cX[1], Z = cX[2];

      e[i * 3] = X - observations[i][0];
      e[i * 3 + 1] = Y - observations[i][1];
      e[i * 3 + 2] = Z - observations[i][2];

      J[i * 3][0] = -1;
      J[i * 3 + 1][1] = -1;
      J[i * 3 + 2][2] = -1;

      J[i * 3][4] = -Z;
      J[i * 3][5] = Y;

      J[i * 3 + 1][3] = Z;
      J[i * 3 + 1][5] = -X;

      J[i * 3 + 2][3] = -Y;
      J[i * 3 + 2][4] = X;
    }
  }

private:
  vpMatrix m_X; // N x 3, points expressed in world frame
  unsigned m_maxPoints;
  double m_minDistNewPoint;
  double m_maxDepthError;
  double m_outlierThreshold;
};

END_VISP_NAMESPACE

#endif
