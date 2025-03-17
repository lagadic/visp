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
 */

#include <visp3/rbt/vpPointMap.h>

BEGIN_VISP_NAMESPACE

void vpPointMap::getPoints(const vpArray2D<int> &indices, vpMatrix &X)
{
  X.resize(indices.getRows(), 3, false, false);
  for (unsigned int i = 0; i < indices.getRows(); ++i) {
    unsigned idx = indices[i][0];
    X[i][0] = m_X[idx][0];
    X[i][1] = m_X[idx][1];
    X[i][2] = m_X[idx][2];
  }
}

void vpPointMap::project(const vpHomogeneousMatrix &cTw, vpMatrix &cX)
{
  cX.resize(m_X.getRows(), 3, false, false);
  vpColVector X(3);
  vpColVector rX(3);

  const vpColVector t = cTw.getTranslationVector();
  const vpRotationMatrix R = cTw.getRotationMatrix();
  for (unsigned int i = 0; i < m_X.getRows(); ++i) {
    X[0] = m_X[i][0];
    X[1] = m_X[i][1];
    X[2] = m_X[i][2];

    rX = R * X;

    cX[i][0] = rX[0] + t[0];
    cX[i][1] = rX[1] + t[1];
    cX[i][2] = rX[2] + t[2];
  }
}

void vpPointMap::project(const vpArray2D<int> &indices, const vpHomogeneousMatrix &cTw, vpMatrix &cX)
{
  cX.resize(indices.getRows(), 3, false, false);
  vpColVector X(3);
  vpColVector rX(3);

  const vpColVector t = cTw.getTranslationVector();
  const vpRotationMatrix R = cTw.getRotationMatrix();
  for (unsigned int i = 0; i < indices.getRows(); ++i) {
    unsigned idx = indices[i][0];
    X[0] = m_X[idx][0];
    X[1] = m_X[idx][1];
    X[2] = m_X[idx][2];

    rX = R * X;

    cX[i][0] = rX[0] + t[0];
    cX[i][1] = rX[1] + t[1];
    cX[i][2] = rX[2] + t[2];
  }
}

void vpPointMap::project(const vpArray2D<int> &indices, const vpHomogeneousMatrix &cTw, vpMatrix &cX, vpMatrix &xs)
{
  project(indices, cTw, cX);
  xs.resize(cX.getRows(), 2, false, false);
  for (unsigned int i = 0; i < cX.getRows(); ++i) {
    xs[i][0] = cX[i][0] / cX[i][2];
    xs[i][1] = cX[i][1] / cX[i][2];
  }
}

void vpPointMap::project(const vpCameraParameters &cam, const vpArray2D<int> &indices, const vpHomogeneousMatrix &cTw, vpMatrix &cX, vpMatrix &xs, vpMatrix &uvs)
{
  if (cam.get_projModel() != vpCameraParameters::vpCameraParametersProjType::perspectiveProjWithoutDistortion) {
    throw vpException(vpException::badValue, "Only cameras without distortion are supported");
  }
  project(indices, cTw, cX, xs);
  uvs.resize(xs.getRows(), xs.getCols(), false, false);
  for (unsigned int i = 0; i < xs.getRows(); ++i) {
    vpMeterPixelConversion::convertPointWithoutDistortion(cam, xs[i][0], xs[i][1], uvs[i][0], uvs[i][1]);
  }
}

void vpPointMap::getVisiblePoints(const unsigned int h, const unsigned int w, const vpMatrix &cX, const vpMatrix &uvs, const vpColVector &expectedZ, std::list<int> &indices)
{
  for (unsigned int i = 0; i < cX.getRows(); ++i) {
    const double u = uvs[i][0], v = uvs[i][1];
    const double Z = cX[i][2];
    if (u < 0 || v < 0 || u >= w || v >= h) {
      continue;
    }
    if (fabs(Z - expectedZ[i]) > m_maxDepthError) {
      continue;
    }
    indices.push_back(i);
  }
}

void vpPointMap::getVisiblePoints(const unsigned int h, const unsigned int w, const vpCameraParameters &cam, const vpHomogeneousMatrix &cTw, const vpImage<float> &depth, std::list<int> &indices)
{
  indices.clear();
  vpColVector cX(3);
  vpColVector oX(3);
  const vpRotationMatrix cRw = cTw.getRotationMatrix();
  const vpTranslationVector t = cTw.getTranslationVector();

  double u, v;
  for (unsigned int i = 0; i < m_X.getRows(); ++i) {
    oX[0] = m_X[i][0];
    oX[1] = m_X[i][1];
    oX[2] = m_X[i][2];

    cX = cRw * oX;
    cX += t;
    const double Z = cX[2];
    if (Z <= 0.0) {
      continue;
    }

    const double x = cX[0] / Z, y = cX[1] / Z;
    vpMeterPixelConversion::convertPointWithoutDistortion(cam, x, y, u, v);
    if (u < 0 || v < 0 || u >= w || v >= h) {
      continue;
    }
    unsigned int uint = static_cast<unsigned int>(u), vint = static_cast<unsigned int>(v);
    if (fabs(Z - depth[vint][uint]) > m_maxDepthError) {
      continue;
    }

    indices.push_back(i);
  }
}

void vpPointMap::getOutliers(const vpArray2D<int> &originalIndices, const vpMatrix &uvs, const vpMatrix &observations, std::list<int> &indices)
{
  if (uvs.getRows() != observations.getRows()) {
    throw vpException(vpException::dimensionError, "Uvs and observations should have same number of rows");
  }
  indices.clear();
  double thresholdSqr = vpMath::sqr(m_outlierThreshold);
  for (unsigned int i = 0; i < uvs.getRows(); ++i) {
    const double error = vpMath::sqr(uvs[i][0] - observations[i][0]) + vpMath::sqr(uvs[i][1] - observations[i][1]);
    if (error >= thresholdSqr) {
      indices.push_back(originalIndices[i][0]);
    }
  }
}

void vpPointMap::selectValidNewCandidates(const vpCameraParameters &cam, const vpHomogeneousMatrix &cTw, const vpArray2D<int> &originalIndices, const vpMatrix &uvs, const vpImage<float> &depth, vpMatrix &oXs, std::list<int> &validCandidateIndices)
{
  if (originalIndices.getRows() != uvs.getRows()) {
    throw vpException(vpException::dimensionError, "Indices and keypoint locations should have the same dimensions");
  }
  validCandidateIndices.clear();
  double x, y;
  vpColVector oX(3);
  vpColVector cX(3);
  const vpHomogeneousMatrix wTc = cTw.inverse();
  const vpRotationMatrix wRc = wTc.getRotationMatrix();
  const vpTranslationVector t = wTc.getTranslationVector();
  double farEnoughThresholdSq = m_minDistNewPoint * m_minDistNewPoint;

  std::vector<std::array<double, 3>> validoXList;
  validoXList.reserve(uvs.getRows());

  for (unsigned int i = 0; i < uvs.getRows(); ++i) {
    double u = uvs[i][0], v = uvs[i][1];
    unsigned int uint = static_cast<unsigned int>(u), vint = static_cast<unsigned int>(v);
    double Z = static_cast<double>(depth[vint][uint]);
    if (Z <= 0.0) {
      continue;
    }

    vpPixelMeterConversion::convertPointWithoutDistortion(cam, u, v, x, y);
    cX[0] = x * Z;
    cX[1] = y * Z;
    cX[2] = Z;
    oX = wRc * cX;
    oX += t;

    // Filter candidates that are too close to already existing points in the map
    bool isFarEnoughFromOtherPoints = true;
    if (m_minDistNewPoint > 0.0) {
      for (unsigned int j = 0; j < m_X.getRows(); ++j) {
        double errSq = vpMath::sqr(oX[0] - m_X[j][0]) + vpMath::sqr(oX[1] - m_X[j][1]) + vpMath::sqr(oX[2] - m_X[j][2]);
        if (errSq < farEnoughThresholdSq) {
          isFarEnoughFromOtherPoints = false;
          break;
        }
      }
      if (isFarEnoughFromOtherPoints) {
        for (const std::array<double, 3> &other: validoXList) {
          double errSq = vpMath::sqr(oX[0] - other[0]) + vpMath::sqr(oX[1] - other[1]) + vpMath::sqr(oX[2] - other[2]);
          if (errSq < farEnoughThresholdSq) {
            isFarEnoughFromOtherPoints = false;
            break;
          }
        }
      }
    }

    if (isFarEnoughFromOtherPoints) {
      validoXList.push_back({ oX[0], oX[1], oX[2] });
      validCandidateIndices.push_back(originalIndices[i][0]);
    }
  }

  oXs.resize(validoXList.size(), 3, false, false);
  unsigned int i = 0;
  for (const std::array<double, 3> &oX: validoXList) {
    oXs[i][0] = oX[0];
    oXs[i][1] = oX[1];
    oXs[i][2] = oX[2];

    ++i;
  }
}

void vpPointMap::updatePoints(const vpArray2D<int> &indicesToRemove, const vpMatrix &pointsToAdd, std::list<int> &removedIndices, unsigned int &numAddedPoints)
{
  removedIndices.clear();
  int newSize = m_X.getRows() - indicesToRemove.getRows() + pointsToAdd.getRows();
  for (unsigned int i = 0; i < indicesToRemove.getRows(); ++i) {
    removedIndices.push_back(indicesToRemove[i][0]);
  }

  removedIndices.sort();
  if (newSize > m_maxPoints) {
    int shouldBeRemoved = newSize - m_maxPoints;
    newSize = m_maxPoints;

    // If the first values are filtered by indicesToRemove, we need to further increment the start index
    std::list<int> startingIndices;
    auto removedIt = removedIndices.begin();
    int i = 0;
    while (startingIndices.size() < shouldBeRemoved && i < m_X.getRows()) {

      if (removedIt == removedIndices.end() || i < (*removedIt)) {
        startingIndices.push_back(i);
      }
      else {
        ++removedIt;
      }
      ++i;
    }

    removedIndices.merge(startingIndices);
  }
  vpMatrix newX(newSize, 3);

  unsigned int newXIndex = 0;
  unsigned int oldXIndex = 0;
  // Copy between removed rows
  for (int removedRow : removedIndices) {
    unsigned int copiedRows = removedRow - oldXIndex;
    if (copiedRows > 0) {
      memcpy(newX[newXIndex], m_X[oldXIndex], copiedRows * 3 * sizeof(double));
      newXIndex += copiedRows;
    }
    oldXIndex = removedRow + 1;
  }
  // Copy from last removed row to the end of the array
  unsigned int copiedRows = m_X.getRows() - oldXIndex;
  if (copiedRows > 0) {
    memcpy(newX[newXIndex], m_X[oldXIndex], copiedRows * 3 * sizeof(double));
    newXIndex += copiedRows;
  }
  numAddedPoints = 0;
  for (unsigned int i = 0; i < pointsToAdd.getRows() && newXIndex < newSize; ++i) {
    for (unsigned int j = 0; j < 3; ++j) {
      newX[newXIndex][j] = pointsToAdd[i][j];
    }
    ++newXIndex;
    ++numAddedPoints;
  }

  m_X = std::move(newX);
}

END_VISP_NAMESPACE
