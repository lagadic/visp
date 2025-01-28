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

#include <map>
#include <set>


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
    m_mapView.update(m_cameras);
  }

  void updateEnvironment(const std::vector<unsigned int> &filteredIndices)
  {
    std::vector<unsigned int> sortedFilteredIndices(filteredIndices);
    std::sort(sortedFilteredIndices.begin(), sortedFilteredIndices.end());
    for (CameraData &camera: m_cameras) {
      camera.filter(sortedFilteredIndices);
    }
    m_mapView.update(m_cameras);
  }

  /**
   * \brief Set the params vector as a flattened view of the optimization parameters.
   *
   * These parameters are:
   * - The camera extrinsic parameters (N x 6 params) in the form (translation, thetaUVector)
   * - The set of 3D points that are seen by at least one camera (M x 3)
   *
   * This results in a N * 6 * M * 3 params, where the N * 6 first values are the camera parameters
   *
   * \param params a 1D vector containing the optimisation variables
  */
  void asParamVector(vpColVector &params);

  std::vector<vpHomogeneousMatrix> getCameraPoses()
  {
    std::vector<vpHomogeneousMatrix> poses;
    for (const CameraData &camera: m_cameras) {
      poses.push_back(vpHomogeneousMatrix(camera.pose()));
    }
    return poses;
  }

  /**
   * \brief Update the camera poses and 3D points from the modified optimisation variables.
   *
   * It should have the same dimension and content as when it was filled with \ref asParamVector
   *
   * \param params
  */
  void updateFromParamVector(const vpColVector &params)
  {

    throw std::runtime_error("updateFromParamVector not implemented");
  }

  void computeError(const vpColVector params, vpColVector &e)
  {

    e.resize(numResiduals(), true);
    unsigned int i = 0;
    unsigned int cameraIndex = 0;
    for (CameraData &camera: m_cameras) {
      camera.error(m_mapView, params, e, cameraIndex, m_cameras.size(), i);
      i += camera.numResiduals();
      ++cameraIndex;
    }
  }

  void jacobianSparsity(vpMatrix &S)
  {
    unsigned int numParams = numCameras() * 6 + numPoints3d() * 3;
    S.resize(numResiduals(), numParams);
    unsigned int i = 0;
    unsigned int cameraIndex = 0;
    for (const CameraData &camera: m_cameras) {
      camera.fillJacobianSparsity(m_mapView, S, cameraIndex, m_cameras.size(), i);
      i += camera.numResiduals();
      ++cameraIndex;
    }

  }

  void computeJacobian(const vpColVector &params, vpMatrix &J)
  {
    // unsigned int numParams = numCameras() * 6 + numPoints3d() * 3;
    // J.resize(numResiduals(), numParams, true, false);
    // unsigned int i = 0;
    // unsigned int cameraIndex = 0;
    // for (CameraData &camera: m_cameras) {
    //   camera.jacobian(params, J, cameraIndex, i);
    //   i += camera.numResiduals();
    //   ++cameraIndex;
    // }
  }


  unsigned int numCameras() const { return m_cameras.size(); }
  unsigned int numPoints3d() const { return m_mapView.numPoints(); }
  unsigned int numResiduals() const
  {
    unsigned int numResiduals = 0;
    for (const CameraData &camera: m_cameras) {
      numResiduals += camera.numResiduals();
    }
    return numResiduals;
  }

  class MapIndexView;

  class CameraData
  {
  public:
    CameraData(const vpCameraParameters &cam, const vpHomogeneousMatrix &cTw, const std::vector<unsigned int> &indices3d, const vpMatrix &uvs)
    {

      m_cTw = cTw;
      m_indices3d = indices3d;

#ifdef DEBUG_RB_BA
      if (m_indices3d.size() != uvs.getRows()) {
        throw vpException(vpException::badValue, "Number of 3D points and 2D observations should be the same!");
      }

      for (unsigned int i = 1; i < m_indices3d.size(); ++i) {
        if (m_indices3d[i] < m_indices3d[i - 1]) {
          throw vpException(vpException::badValue, "3D index list should be sorted!");
        }
      }
#endif

      m_points2d.resize(uvs.getRows());

      for (unsigned int i = 0; i < uvs.getRows(); ++i) {
        vpPixelMeterConversion::convertPointWithoutDistortion(cam, uvs[i][0], uvs[i][1], m_points2d[i][0], m_points2d[i][1]);
      }
    }

    void fillJacobianSparsity(const MapIndexView &mapView, vpMatrix &S, unsigned int cameraIndex, unsigned int numCameras, unsigned int startResidual) const;

    vpPoseVector pose() const { return vpPoseVector(m_cTw); }
    void setPose(const vpPoseVector &r) { m_cTw = r; }

    void error(MapIndexView &mapView, const vpColVector &params, vpColVector &e, unsigned int cameraIndex, unsigned int numCameras, unsigned int startResidual) const;

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
      m_points2d = std::move(newPoints2d);


#ifdef DEBUG_RB_BA
      if (m_indices3d.size() != m_points2d.size()) {
        throw vpException(vpException::badValue, "Number of 3D points and 2D observations should be the same!");
      }
#endif

    }

    const std::vector<unsigned int> &getPointsIndices() const { return m_indices3d; }

  private:
    vpHomogeneousMatrix m_cTw;
    std::vector<unsigned int> m_indices3d;
    std::vector<std::array<double, 2>> m_points2d;

  };


  /**
   * \brief Helper class to associate a 3D point to its location in the parameters vector and the Jacobian Matrix
   * Since not all map points may be used during the optimisation, This class will help ensure that no zero column appears in the Jacobian
   * Reducing the complexity of the of the optimisation.
   *
   * Only points that are seen by at least one camera will be stored in this map. It should be updated every time the map or cameras change.
   *
  */
  class MapIndexView
  {
  public:
    MapIndexView() = default;

    void update(const std::list<CameraData> &cameras);

    inline unsigned int numPoints() const { return m_pointToView.size(); }
    inline unsigned int getPointIndex(unsigned int viewIndex) const { return m_viewToPoint.find(viewIndex)->second; }
    inline unsigned int getViewIndex(unsigned int pointIndex) const { return m_pointToView.find(pointIndex)->second; }

  private:
    std::map<unsigned int, unsigned int> m_pointToView;
    std::map<unsigned int, unsigned int> m_viewToPoint;

  };

private:
  vpCameraParameters m_cam;
  unsigned int m_numCams;
  vpPointMap *m_map;
  std::list<CameraData> m_cameras;
  MapIndexView m_mapView;




};


END_VISP_NAMESPACE

#endif
