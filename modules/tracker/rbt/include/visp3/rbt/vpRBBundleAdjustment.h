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
  \file vpRBBundleAdjustment.h
  \brief A simple bundle adjustment problem resolution helper
*/
#ifndef VP_RB_BUNDLE_ADJUSTMENT_H
#define VP_RB_BUNDLE_ADJUSTMENT_H

#include <visp3/core/vpConfig.h>

#include <map>

#include <visp3/rbt/vpRBFeatureTracker.h>
#include <visp3/core/vpPoint.h>
#include <visp3/core/vpTrackingException.h>
#include <visp3/core/vpRobust.h>
#include <visp3/rbt/vpPointMap.h>

BEGIN_VISP_NAMESPACE

class VISP_EXPORT vpRBBundleAdjustment
{
public:
  vpRBBundleAdjustment(unsigned int numCams, const vpCameraParameters &cam, vpPointMap &map);

  void addNewCamera(const vpHomogeneousMatrix &cTw, const std::vector<unsigned int> &indices3d, const vpMatrix &uvs);

  void updateEnvironment(const std::vector<unsigned int> &filteredIndices);
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
  void asParamVector(vpColVector &params) const;

  std::vector<vpHomogeneousMatrix> getCameraPoses() const;

  /**
   * \brief Update the camera poses and 3D points from the modified optimisation variables.
   *
   * It should have the same dimension and content as when it was filled with \ref asParamVector
   *
   * \param params
  */
  void updateFromParamVector(const vpColVector &params);

  /**
   * @brief Compute the BA residuals, which are the reprojection errors between the projection of a 3D in a given camera with their associated 2D observation
   * The errors are computed in normalized coordinate space since we assume that the camera intrinsics are the same for all cameras and are perfectly known.
   *
   * @param params The parameter vector, from which we compute the residuals. It should have been computed using asParamVector, and updated with an optimization algorithm
   * @param e the residual vector, the reprojection errors.
   */
  void computeError(const vpColVector &params, vpColVector &e);

  /**
   * @brief Compute the Jacobian sparsity matrix, that indicates which parameters are used when dealing with a given residual error
   *
   * @param S The Jacobian sparsity matrix, that has the same size as the jacobian
   */
  void jacobianSparsity(vpArray2D<int> &S);

  void computeJacobian(const vpColVector &params, vpMatrix &J)
  {
    unsigned int numParams = numCameras() * 6 + numPoints3d() * 3;
    J.resize(numResiduals(), numParams, true, false);
    unsigned int i = 0;
    unsigned int cameraIndex = 0;
    for (CameraData &camera: m_cameras) {
      camera.jacobian(m_mapView, params, J, cameraIndex, numCameras(), i);
      i += camera.numResiduals();
      ++cameraIndex;
    }
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
    CameraData(const vpCameraParameters &cam, const vpHomogeneousMatrix &cTw, const std::vector<unsigned int> &indices3d, const vpMatrix &uvs);


    unsigned int numResiduals() const { return m_points2d.size() * 2; }
    const std::vector<unsigned int> &getPointsIndices() const { return m_indices3d; }
    vpPoseVector pose() const { return vpPoseVector(m_cTw); }
    void setPose(const vpPoseVector &r) { m_cTw = r; }

    void error(MapIndexView &mapView, const vpColVector &params, vpColVector &e, unsigned int cameraIndex, unsigned int numCameras, unsigned int startResidual) const;
    void jacobian(const MapIndexView &mapView, const vpColVector &params, vpMatrix &J, unsigned int cameraIndex, unsigned int numCameras, unsigned int startResidual) const;
    void fillJacobianSparsity(const MapIndexView &mapView, vpArray2D<int> &S, unsigned int cameraIndex, unsigned int numCameras, unsigned int startResidual) const;


    void filter(const std::vector<unsigned int> &filteredIndices);
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
   * Only points that are seen by at least one camera will be stored in this map. It should be updated every time the map or the cameras change.
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
  vpCameraParameters m_cam; //! Camera intrinsics associated with the considered camera poses
  unsigned int m_numCams; // Max number of cameras that can be considered in the system
  vpPointMap *m_map; // Map containing the 3D points that can be associated to 2D observations
  std::list<CameraData> m_cameras; //! Data for each camera to optimize
  MapIndexView m_mapView; //! Helper object to restrain the optimization variables to points that are observed by at least one camera
};


END_VISP_NAMESPACE

#endif
