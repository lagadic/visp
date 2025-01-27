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

/*!
  \file vpRBKltTracker.h
  \brief KLT features in the context of render based tracking
*/
#ifndef VP_RB_BUNDLE_ADJUSTMENT_H
#define VP_RB_BUNDLE_ADJUSTMENT_H

#define DEBUG_RB_BA 1

#include <visp3/core/vpConfig.h>


#include <visp3/rbt/vpRBFeatureTracker.h>
#include <visp3/core/vpPoint.h>
#include <visp3/core/vpTrackingException.h>
#include <visp3/core/vpRobust.h>
#include <visp3/rbt/vpPointMap.h>


BEGIN_VISP_NAMESPACE

class VISP_EXPORT vpRBBundleAdjustment
{
public:



  vpRBBundleAdjustment(unsigned int numCams, const vpCameraParameters &cam, vpPointMap &map)
  {
    m_numCams = numCams;
    m_cam = cam;
    m_map = &map;
  }

  void addNewCamera(const vpHomogeneousMatrix &cTw, const std::vector<unsigned int> &indices3d, const vpMatrix &uvs)
  {
    if (m_cameras.size() == m_numCams) {
      m_cameras.pop_front();
    }
    m_cameras.push_back(CameraData(m_cam, cTw, indices3d, uvs));
  }

  void updateEnvironment(const std::vector<unsigned int> &filteredIndices)
  {
    std::vector<unsigned int> sortedFilteredIndices(filteredIndices);
    std::sort(sortedFilteredIndices.begin(), sortedFilteredIndices.end());
    for (CameraData &camera: m_cameras) {
      camera.filter(sortedFilteredIndices);
    }
  }

  void asParamVector(vpColVector &e)
  {
    throw std::runtime_error("asparam not implemented");
  }

  void updateFromParamVector(const vpColVector &params)
  {
    throw std::runtime_error("updateFromParamVector not implemented");
  }

  void computeError(vpColVector &e)
  {

    e.resize(numResiduals(), false);

    unsigned int i = 0;
    for (CameraData &camera: m_cameras) {
      camera.error(e, i);
    }
  }

  void computeJacobian(vpMatrix &J)
  {
    unsigned int numParams = numCameras() * 6 + numPoints3d() * 3;
    J.resize(numResiduals(), numParams, true, false);
    unsigned int i = 0;
    for (unsigned int cameraIndex = 0; cameraIndex < m_cameras.size(); ++cameraIndex) {
      camera.jacobian(J, cameraIndex, i);
    }
  }


  unsigned int numCameras() const { return m_cameras.size(); }
  unsigned int numPoints3d() const { return m_map->getPoints().getRows(); }
  unsigned int numResiduals() const
  {
    unsigned int numResiduals = 0;
    for (const CameraData &camera: m_cameras) {
      numResiduals += camera.numResiduals();
    }
  }


  class CameraData
  {
  public:
    CameraData(const vpCameraParameters &cam, const vpHomogeneousMatrix &cTw, const std::vector<unsigned int> &indices3d, const vpMatrix &uvs)
    {

      m_cTw = cTw;
      m_indices3d = indices3d;

#ifdef DEBUG_RB_BA
      for (unsigned int i = 1; i < m_indices3d.size(); ++i) {
        if (m_indices3d[i] < m_indices3d[i - 1]) {
          throw vpException(vpException::badValue, "3D index list should be sorted!");
        }
      }
#endif

      m_points2d.resize(uvs.getRows());

      for (unsigned int i = 0; i < uvs.getRows(); ++i) {
        vpPixelMeterConversion::convertPoint(cam, uvs[i][0], uvs[i][1], m_points2d[i][0], m_points2d[i][1]);
      }
    }



    unsigned int numResiduals() const { return m_points2d.size() * 2; }

    void filter(const std::vector<unsigned int> &filteredIndices)
    {
#ifdef DEBUG_RB_BA
      for (unsigned int i = 1; i < filteredIndices.size(); ++i) {
        if (filteredIndices[i] < filteredIndices[i - 1]) {
          throw vpException(vpException::badValue, "Removed indices should be sorted!");
        }
      }
#endif
      std::vector<unsigned int> indicestoKeep;
      unsigned int currentFilterIndex = 0;
      unsigned int currentIndex = 0;
      while (currentIndex < m_indices3d.size() && currentFilterIndex < filteredIndices.size()) {
        unsigned int toRemove = filteredIndices[currentFilterIndex];
        unsigned int currentValue = m_indices3d[currentIndex];
        while (currentValue < toRemove) {
          indicestoKeep.push_back(currentIndex);
          ++currentIndex;
          if (currentIndex > m_indices3d.size()) {
            break;
          }
          currentValue = m_indices3d[currentIndex];
        }

        ++currentFilterIndex;
      }

      std::vector<std::array<double, 2>> newPoints2d(indicestoKeep.size());
      for (unsigned int i = 0; i < indicestoKeep.size(); ++i) {
        newPoints2d[i][0] = m_points2d[indicestoKeep[i]][0];
        newPoints2d[i][1] = m_points2d[indicestoKeep[i]][1];
      }

      m_indices3d = std::move(indicestoKeep);
    }
  private:
    vpHomogeneousMatrix m_cTw;
    std::vector<unsigned int> m_indices3d;
    std::vector<std::array<double, 2>> m_points2d;

  };

private:
  vpCameraParameters m_cam;
  unsigned int m_numCams;
  vpPointMap *m_map;
  std::list<CameraData> m_cameras;




};


END_VISP_NAMESPACE

#endif
#endif
