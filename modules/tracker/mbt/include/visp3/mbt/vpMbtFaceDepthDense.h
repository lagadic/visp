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
 * Manage depth dense features for a particular face.
 *
 *****************************************************************************/

#ifndef __vpMbtFaceDepthDense_h_
#define __vpMbtFaceDepthDense_h_

#include <iostream>

#include <visp3/core/vpConfig.h>
#ifdef VISP_HAVE_PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#endif

#include <visp3/core/vpPlane.h>
#include <visp3/mbt/vpMbTracker.h>
#include <visp3/mbt/vpMbtDistanceLine.h>

#define DEBUG_DISPLAY_DEPTH_DENSE 0

class VISP_EXPORT vpMbtFaceDepthDense
{
public:
  enum vpDepthDenseFilteringType {
    NO_FILTERING = 0,                         ///< Face is used if visible
    DEPTH_OCCUPANCY_RATIO_FILTERING = 1 << 1, ///< Face is used if there is
                                              ///< enough depth information in
                                              ///< the face polygon
    MIN_DISTANCE_FILTERING = 1 << 2,          ///< Face is used if the camera position
                                              ///< is farther than the threshold
    MAX_DISTANCE_FILTERING = 1 << 3           ///< Face is used if the camera position
                                              ///< is closer than the threshold
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

  vpMbtFaceDepthDense();
  virtual ~vpMbtFaceDepthDense();

  void addLine(vpPoint &p1, vpPoint &p2, vpMbHiddenFaces<vpMbtPolygon> *const faces, int polygon = -1,
               std::string name = "");

#ifdef VISP_HAVE_PCL
  bool computeDesiredFeatures(const vpHomogeneousMatrix &cMo,
                              const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &point_cloud, const unsigned int stepX,
                              const unsigned int stepY
#if DEBUG_DISPLAY_DEPTH_DENSE
                              ,
                              vpImage<unsigned char> &debugImage, std::vector<std::vector<vpImagePoint> > &roiPts_vec
#endif
  );
#endif
  bool computeDesiredFeatures(const vpHomogeneousMatrix &cMo, const unsigned int width, const unsigned int height,
                              const std::vector<vpColVector> &point_cloud, const unsigned int stepX,
                              const unsigned int stepY
#if DEBUG_DISPLAY_DEPTH_DENSE
                              ,
                              vpImage<unsigned char> &debugImage, std::vector<std::vector<vpImagePoint> > &roiPts_vec
#endif
  );

  void computeInteractionMatrixAndResidu(const vpHomogeneousMatrix &cMo, vpMatrix &L, vpColVector &error);

  void computeVisibility();
  void computeVisibilityDisplay();

  void display(const vpImage<unsigned char> &I, const vpHomogeneousMatrix &cMo, const vpCameraParameters &cam,
               const vpColor &col, const unsigned int thickness = 1, const bool displayFullModel = false);
  void display(const vpImage<vpRGBa> &I, const vpHomogeneousMatrix &cMo, const vpCameraParameters &cam,
               const vpColor &col, const unsigned int thickness = 1, const bool displayFullModel = false);

  void displayFeature(const vpImage<unsigned char> &I, const vpHomogeneousMatrix &cMo, const vpCameraParameters &cam,
                      const double scale = 0.05, const unsigned int thickness = 1);
  void displayFeature(const vpImage<vpRGBa> &I, const vpHomogeneousMatrix &cMo, const vpCameraParameters &cam,
                      const double scale = 0.05, const unsigned int thickness = 1);

  inline unsigned int getNbFeatures() const { return (unsigned int)(m_pointCloudFace.size() / 3); }

  inline bool isVisible() const { return m_polygon->isvisible; }

  void setCameraParameters(const vpCameraParameters &camera);

  void setScanLineVisibilityTest(const bool v);

  inline void setDepthDenseFilteringMaxDistance(const double maxDistance)
  {
    m_depthDenseFilteringMaxDist = maxDistance;
  }

  inline void setDepthDenseFilteringMethod(const int method) { m_depthDenseFilteringMethod = method; }

  inline void setDepthDenseFilteringMinDistance(const double minDistance)
  {
    m_depthDenseFilteringMinDist = minDistance;
  }

  inline void setDepthDenseFilteringOccupancyRatio(const double occupancyRatio)
  {
    if (occupancyRatio < 0.0 || occupancyRatio > 1.0) {
      std::cerr << "occupancyRatio < 0.0 || occupancyRatio > 1.0" << std::endl;
    } else {
      m_depthDenseFilteringOccupancyRatio = occupancyRatio;
    }
  }

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

protected:
  //! Method to use to consider or not the face
  int m_depthDenseFilteringMethod;
  //! Maximum distance threshold
  double m_depthDenseFilteringMaxDist;
  //! Minimum distance threshold
  double m_depthDenseFilteringMinDist;
  //! Ratio between available depth points and theoretical number of points
  double m_depthDenseFilteringOccupancyRatio;
  //! Flag to define if the face should be tracked or not
  bool m_isTracked;
  //! Visibility flag
  bool m_isVisible;
  std::vector<vpMbtDistanceLine *> m_listOfFaceLines;
  //! Plane equation described in the camera frame and updated with the
  //! current pose
  vpPlane m_planeCamera;
  //! List of depth points inside the face
  std::vector<double> m_pointCloudFace;
  //! Polygon lines used for scan-line visibility
  std::vector<PolygonLine> m_polygonLines;

protected:
  void computeROI(const vpHomogeneousMatrix &cMo, const unsigned int width, const unsigned int height,
                  std::vector<vpImagePoint> &roiPts
#if DEBUG_DISPLAY_DEPTH_DENSE
                  ,
                  std::vector<std::vector<vpImagePoint> > &roiPts_vec
#endif
                  ,
                  double &distanceToFace);

  bool samePoint(const vpPoint &P1, const vpPoint &P2) const;
};
#endif
