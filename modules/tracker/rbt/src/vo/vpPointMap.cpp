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

#ifdef VISP_HAVE_OPENMP
#include <omp.h>
#endif

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

  const vpTranslationVector t = cTw.getTranslationVector();
  const vpRotationMatrix R = cTw.getRotationMatrix();

  vpMatrix::mult2Matrices(m_X, R.t(), cX);
  for (unsigned int i = 0; i < m_X.getRows(); ++i) {
    cX[i][0] += t[0];
    cX[i][1] += t[1];
    cX[i][2] += t[2];
  }
}

void vpPointMap::project(const vpArray2D<int> &indices, const vpHomogeneousMatrix &cTw, vpMatrix &cX)
{
  vpMatrix X(indices.getRows(), 3);
  for (unsigned int i = 0; i < indices.getRows(); ++i) {
    unsigned idx = indices[i][0];
    X[i][0] = m_X[idx][0];
    X[i][1] = m_X[idx][1];
    X[i][2] = m_X[idx][2];
  }
  cX.resize(indices.getRows(), 3, false, false);

  const vpTranslationVector t = cTw.getTranslationVector();
  const vpRotationMatrix R = cTw.getRotationMatrix();

  vpMatrix::mult2Matrices(X, R.t(), cX);

  for (unsigned int i = 0; i < indices.getRows(); ++i) {
    cX[i][0] += t[0];
    cX[i][1] += t[1];
    cX[i][2] += t[2];
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

void vpPointMap::getVisiblePoints(const unsigned int h, const unsigned int w, const vpMatrix &cX, const vpMatrix &uvs, const vpColVector &expectedZ, std::vector<int> &indices)
{
  for (unsigned int i = 0; i < cX.getRows(); ++i) {
    const double u = uvs[i][0], v = uvs[i][1];
    const double Z = cX[i][2];
    if (u < 0 || v < 0 || u >= w || v >= h) {
      continue;
    }
    if (fabs(Z - expectedZ[i]) > m_maxDepthErrorVisible) {
      continue;
    }
    indices.push_back(i);
  }
}

void vpPointMap::getVisiblePoints(const unsigned int h, const unsigned int w, const vpCameraParameters &cam, const vpHomogeneousMatrix &cTw, const vpImage<float> &depth, std::vector<int> &indices)
{
  indices.clear();
  const vpRotationMatrix cRw = cTw.getRotationMatrix();
  const vpTranslationVector t = cTw.getTranslationVector();
  vpMatrix cX(m_X.getRows(), m_X.getCols());
  vpMatrix::mult2Matrices(m_X, cRw.t(), cX);


#ifdef VISP_HAVE_OPENMP
#pragma omp parallel
#endif
  {
    std::vector<int> localIndices;
    double u, v;

#ifdef VISP_HAVE_OPENMP
    localIndices.reserve(m_X.getRows() / omp_get_num_threads());
#pragma omp for nowait
#endif
    for (unsigned int i = 0; i < m_X.getRows(); ++i) {
      const double Z = cX[i][2] + t[2];

      if (Z <= 0.0) {
        continue;
      }
      const double X = cX[i][0] + t[0], Y = cX[i][1] + t[1];
      const double x = X / Z, y = Y / Z;
      vpMeterPixelConversion::convertPointWithoutDistortion(cam, x, y, u, v);
      if (u < 0 || v < 0 || u >= w || v >= h) {
        continue;
      }
      unsigned int uint = static_cast<unsigned int>(u), vint = static_cast<unsigned int>(v);
      if (fabs(Z - depth[vint][uint]) > m_maxDepthErrorVisible) {
        continue;
      }

      localIndices.push_back(i);
    }
#ifdef VISP_HAVE_OPENMP
#pragma omp critical
#endif
    {
      indices.insert(indices.end(), std::make_move_iterator(localIndices.begin()), std::make_move_iterator(localIndices.end()));
    }
  }
}

void vpPointMap::getOutliers(const vpArray2D<int> &originalIndices, const vpMatrix &uvs, const vpMatrix &observations, std::vector<int> &indices)
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

void vpPointMap::selectValidNewCandidates(const vpCameraParameters &cam, const vpHomogeneousMatrix &cTw, const vpArray2D<int> &originalIndices,
const vpMatrix &uvs, const vpImage<float> &modelDepth, const vpImage<float> &depth,
vpMatrix &oXs, std::vector<int> &validCandidateIndices)
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
    double Z;
    if (modelDepth.getSize() == 0) { // We are performing odometry or do not have a depth oracle
      Z = static_cast<double>(depth[vint][uint]);
      if (Z <= 0.0) {
        continue;
      }
    }
    else {
      double renderZ = modelDepth[vint][uint];
      if (renderZ <= 0.f) {
        continue;
      }
      if (depth.getSize() > 0 && depth[vint][uint] > 0.f) { // Depth information from camera is available
        Z = depth[vint][uint];
        // Check if depth from model and camera match
        if (m_maxDepthErrorCandidate > 0.0 && fabs(renderZ - Z) >=  m_maxDepthErrorCandidate) {
          continue;
        }
      }
      Z = renderZ; // For addition, use the rendered depth
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

void vpPointMap::clear()
{
  m_X = vpMatrix();
}

void vpPointMap::updatePoints(const vpArray2D<int> &indicesToRemove, const vpMatrix &pointsToAdd,
                              std::vector<int> &removedIndices, unsigned int &numAddedPoints)
{
  removedIndices.clear();
  int newSize = m_X.getRows() - indicesToRemove.getRows() + pointsToAdd.getRows();
  for (unsigned int i = 0; i < indicesToRemove.getRows(); ++i) {
    removedIndices.push_back(indicesToRemove[i][0]);
  }

  int maxPoints = static_cast<int>(m_maxPoints);
  std::sort(removedIndices.begin(), removedIndices.end());
  if (newSize > maxPoints) {
    int shouldBeRemoved = newSize - maxPoints;
    newSize = maxPoints;

    // If the first values are filtered by indicesToRemove, we need to further increment the start index
    std::vector<int> startingIndices;
    auto removedIt = removedIndices.begin();
    int i = 0;
    int n_rows = static_cast<int>(m_X.getRows());
    while ((startingIndices.size() < static_cast<size_t>(shouldBeRemoved)) && (i < n_rows)) {

      if (removedIt == removedIndices.end() || i < (*removedIt)) {
        startingIndices.push_back(i);
      }
      else {
        ++removedIt;
      }
      ++i;
    }

    removedIndices.insert(removedIndices.begin(), startingIndices.begin(), startingIndices.end());
    std::sort(removedIndices.begin(), removedIndices.end());
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

  numAddedPoints = std::min(pointsToAdd.getRows(), static_cast<unsigned int>(newSize) - newXIndex);
  memcpy(newX[newXIndex], pointsToAdd[0], numAddedPoints * 3 * sizeof(double));

  m_X = std::move(newX);
}

END_VISP_NAMESPACE
