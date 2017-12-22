/****************************************************************************
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2017 by Inria. All rights reserved.
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
 * Manage depth normal features for a particular face.
 *
 *****************************************************************************/

#ifndef __vpMbtFaceDepthNormal_h_
#define __vpMbtFaceDepthNormal_h_

#include <iostream>

#include <visp3/core/vpConfig.h>
#ifdef VISP_HAVE_PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#endif

#include <visp3/core/vpPlane.h>
#include <visp3/mbt/vpMbTracker.h>
#include <visp3/mbt/vpMbtDistanceLine.h>

#define DEBUG_DISPLAY_DEPTH_NORMAL 0

class VISP_EXPORT vpMbtFaceDepthNormal
{
public:
  enum vpFaceCentroidType {
    GEOMETRIC_CENTROID, ///< Compute the geometric centroid
    MEAN_CENTROID       ///< Compute the mean centroid
  };

  enum vpFeatureEstimationType {
    ROBUST_FEATURE_ESTIMATION = 0,
    ROBUST_SVD_PLANE_ESTIMATION = 1,
#ifdef VISP_HAVE_PCL
    PCL_PLANE_ESTIMATION = 2
#endif
  };

  //! Camera intrinsic parameters
  vpCameraParameters m_cam;
  //! Flags specifying which clipping to used
  unsigned int m_clippingFlag;
  //! Distance for near clipping
  double m_distFarClip;
  //! Distance for near clipping
  double m_distNearClip;
  //! Pointer to the list of faces
  vpMbHiddenFaces<vpMbtPolygon> *m_hiddenFace;
  //! Plane equation described in the object frame
  vpPlane m_planeObject;
  //! Polygon defining the face
  vpMbtPolygon *m_polygon;
  //! Scan line visibility
  bool m_useScanLine;

  vpMbtFaceDepthNormal();
  virtual ~vpMbtFaceDepthNormal();

  void addLine(vpPoint &p1, vpPoint &p2, vpMbHiddenFaces<vpMbtPolygon> *const faces, int polygon = -1,
               std::string name = "");

#ifdef VISP_HAVE_PCL
  bool computeDesiredFeatures(const vpHomogeneousMatrix &cMo, const unsigned int width, const unsigned int height,
                              const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &point_cloud,
                              vpColVector &desired_features, const unsigned int stepX, const unsigned int stepY
#if DEBUG_DISPLAY_DEPTH_NORMAL
                              ,
                              vpImage<unsigned char> &debugImage, std::vector<std::vector<vpImagePoint> > &roiPts_vec
#endif
  );
#endif
  bool computeDesiredFeatures(const vpHomogeneousMatrix &cMo, const unsigned int width, const unsigned int height,
                              const std::vector<vpColVector> &point_cloud, vpColVector &desired_features,
                              const unsigned int stepX, const unsigned int stepY
#if DEBUG_DISPLAY_DEPTH_NORMAL
                              ,
                              vpImage<unsigned char> &debugImage, std::vector<std::vector<vpImagePoint> > &roiPts_vec
#endif
  );

  void computeInteractionMatrix(const vpHomogeneousMatrix &cMo, vpMatrix &L, vpColVector &features);

  void computeVisibility();
  void computeVisibilityDisplay();

  void computeNormalVisibility(const double nx, const double ny, const double nz, const vpColVector &centroid_point,
                               vpColVector &face_normal);
#ifdef VISP_HAVE_PCL
  void computeNormalVisibility(const float nx, const float ny, const float nz, const pcl::PointXYZ &centroid_point,
                               pcl::PointXYZ &face_normal);
#endif
  void computeNormalVisibility(const double nx, const double ny, const double nz, const vpHomogeneousMatrix &cMo,
                               const vpCameraParameters &camera, vpColVector &correct_normal, vpPoint &centroid);

  void display(const vpImage<unsigned char> &I, const vpHomogeneousMatrix &cMo, const vpCameraParameters &cam,
               const vpColor &col, const unsigned int thickness = 1, const bool displayFullModel = false);
  void display(const vpImage<vpRGBa> &I, const vpHomogeneousMatrix &cMo, const vpCameraParameters &cam,
               const vpColor &col, const unsigned int thickness = 1, const bool displayFullModel = false);

  void displayFeature(const vpImage<unsigned char> &I, const vpHomogeneousMatrix &cMo, const vpCameraParameters &cam,
                      const double scale = 0.05, const unsigned int thickness = 1);
  void displayFeature(const vpImage<vpRGBa> &I, const vpHomogeneousMatrix &cMo, const vpCameraParameters &cam,
                      const double scale = 0.05, const unsigned int thickness = 1);

  inline bool isVisible() const { return m_polygon->isvisible; }

  void setCameraParameters(const vpCameraParameters &camera);

  inline void setFaceCentroidMethod(const vpFaceCentroidType &method) { m_faceCentroidMethod = method; }

  inline void setFeatureEstimationMethod(const vpFeatureEstimationType &method) { m_featureEstimationMethod = method; }

  inline void setPclPlaneEstimationMethod(const int method) { m_pclPlaneEstimationMethod = method; }

  inline void setPclPlaneEstimationRansacMaxIter(const int maxIter) { m_pclPlaneEstimationRansacMaxIter = maxIter; }

  inline void setPclPlaneEstimationRansacThreshold(const double threshold)
  {
    m_pclPlaneEstimationRansacThreshold = threshold;
  }

  void setScanLineVisibilityTest(const bool v);

private:
  class PolygonLine
  {
  public:
    //! The first extremity
    vpPoint *m_p1;
    //! The second extremity
    vpPoint *m_p2;
    //! Polygon describing the line
    vpMbtPolygon m_poly;
    //! The first extremity clipped in the image frame
    vpImagePoint m_imPt1;
    //! The second extremity clipped in the image frame
    vpImagePoint m_imPt2;

    PolygonLine() : m_p1(NULL), m_p2(NULL), m_poly(), m_imPt1(), m_imPt2() {}

    PolygonLine(const PolygonLine &polyLine)
      : m_p1(NULL), m_p2(NULL), m_poly(polyLine.m_poly), m_imPt1(polyLine.m_imPt1), m_imPt2(polyLine.m_imPt2)
    {
      m_p1 = &m_poly.p[0];
      m_p2 = &m_poly.p[1];
    }

    PolygonLine &operator=(PolygonLine other)
    {
      swap(*this, other);

      return *this;
    }

    void swap(PolygonLine &first, PolygonLine &second)
    {
      using std::swap;
      swap(first.m_p1, second.m_p1);
      swap(first.m_p2, second.m_p2);
      swap(first.m_poly, second.m_poly);
      swap(first.m_imPt1, second.m_imPt1);
      swap(first.m_imPt2, second.m_imPt2);
    }
  };

  template <class T> class Mat33
  {
  public:
    std::vector<T> data;

    Mat33() : data(9) {}

    inline T operator[](const size_t i) const { return data[i]; }

    inline T &operator[](const size_t i) { return data[i]; }

    Mat33 inverse() const
    {
      // determinant
      T det = data[0] * (data[4] * data[8] - data[7] * data[5]) - data[1] * (data[3] * data[8] - data[5] * data[6]) +
              data[2] * (data[3] * data[7] - data[4] * data[6]);
      T invdet = 1 / det;

      Mat33<T> minv;
      minv[0] = (data[4] * data[8] - data[7] * data[5]) * invdet;
      minv[1] = (data[2] * data[7] - data[1] * data[8]) * invdet;
      minv[2] = (data[1] * data[5] - data[2] * data[4]) * invdet;
      minv[3] = (data[5] * data[6] - data[3] * data[8]) * invdet;
      minv[4] = (data[0] * data[8] - data[2] * data[6]) * invdet;
      minv[5] = (data[3] * data[2] - data[0] * data[5]) * invdet;
      minv[6] = (data[3] * data[7] - data[6] * data[4]) * invdet;
      minv[7] = (data[6] * data[1] - data[0] * data[7]) * invdet;
      minv[8] = (data[0] * data[4] - data[3] * data[1]) * invdet;

      return minv;
    }
  };

protected:
  //! True if the face should be considered by the tracker
  bool m_faceActivated;
  //! Method to compute the face centroid for the current features
  vpFaceCentroidType m_faceCentroidMethod;
  //! Desired centroid (computed from the sensor)
  vpPoint m_faceDesiredCentroid;
  //! Face (normalized) normal (computed from the sensor)
  vpPoint m_faceDesiredNormal;
  //! Method to estimate the desired features
  vpFeatureEstimationType m_featureEstimationMethod;
  //!
  bool m_isTracked;
  //!
  bool m_isVisible;
  //!
  std::vector<vpMbtDistanceLine *> m_listOfFaceLines;
  //! Plane equation described in the camera frame and updated with the
  //! current pose
  vpPlane m_planeCamera;
  //! PCL plane estimation method
  int m_pclPlaneEstimationMethod;
  //! PCL pane estimation max number of iterations
  int m_pclPlaneEstimationRansacMaxIter;
  //! PCL plane estimation RANSAC threshold
  double m_pclPlaneEstimationRansacThreshold;
  //!
  std::vector<PolygonLine> m_polygonLines;

#ifdef VISP_HAVE_PCL
  bool computeDesiredFeaturesPCL(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &point_cloud_face,
                                 vpColVector &desired_features, vpColVector &desired_normal,
                                 vpColVector &centroid_point);
#endif
  void computeDesiredFeaturesRobustFeatures(const std::vector<double> &point_cloud_face_custom,
                                            const std::vector<double> &point_cloud_face, const vpHomogeneousMatrix &cMo,
                                            vpColVector &desired_features, vpColVector &desired_normal,
                                            vpColVector &centroid_point);
  void computeDesiredFeaturesSVD(const std::vector<double> &point_cloud_face, const vpHomogeneousMatrix &cMo,
                                 vpColVector &desired_features, vpColVector &desired_normal,
                                 vpColVector &centroid_point);
  void computeDesiredNormalAndCentroid(const vpHomogeneousMatrix &cMo, const vpColVector &desired_normal,
                                       const vpColVector &centroid_point);

  bool computePolygonCentroid(const std::vector<vpPoint> &points, vpPoint &centroid);

  void computeROI(const vpHomogeneousMatrix &cMo, const unsigned int width, const unsigned int height,
                  std::vector<vpImagePoint> &roiPts
#if DEBUG_DISPLAY_DEPTH_NORMAL
                  ,
                  std::vector<std::vector<vpImagePoint> > &roiPts_vec
#endif
  );

  void estimateFeatures(const std::vector<double> &point_cloud_face, const vpHomogeneousMatrix &cMo,
                        vpColVector &x_estimated, std::vector<double> &weights);

  void estimatePlaneEquationSVD(const std::vector<double> &point_cloud_face, const vpHomogeneousMatrix &cMo,
                                vpColVector &plane_equation_estimated, vpColVector &centroid);

  bool samePoint(const vpPoint &P1, const vpPoint &P2) const;
};
#endif
