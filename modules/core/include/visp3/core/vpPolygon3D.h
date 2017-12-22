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
 * Implements a polygon of the model used by the model-based tracker.
 *
 * Authors:
 * Aurelien Yol
 *
 *****************************************************************************/

/*!
 \file vpPolygon3D.h
 \brief Implements a 3D polygon with render functionnalities like clipping.
*/

#ifndef vpPolygon3D_HH
#define vpPolygon3D_HH

#include <vector>

#include <visp3/core/vpMeterPixelConversion.h>
#include <visp3/core/vpPixelMeterConversion.h>
#include <visp3/core/vpPoint.h>

/*!
  \class vpPolygon3D
  \ingroup group_core_geometry
  \brief Implements a 3D polygon with render functionnalities like clipping.

*/
class VISP_EXPORT vpPolygon3D
{
public:
  typedef enum {
    NO_CLIPPING = 0,
    NEAR_CLIPPING = 1,
    FAR_CLIPPING = 2,
    LEFT_CLIPPING = 4,
    RIGHT_CLIPPING = 8,
    UP_CLIPPING = 16,
    DOWN_CLIPPING = 32,
    FOV_CLIPPING = 60,
    ALL_CLIPPING = 63
  } vpPolygon3DClippingType;

public:
  //! Number of points used to define the polygon.
  unsigned int nbpt;
  //! Number of corners inside the image during the last call to
  //! getNbCornerInsideImage
  unsigned int nbCornersInsidePrev;
  //! corners in the object frame
  vpPoint *p;
  //! Region of interest clipped
  std::vector<std::pair<vpPoint, unsigned int> > polyClipped;
  //! Clipping flag
  unsigned int clippingFlag;
  //! Distance for near clipping
  double distNearClip;
  //! Distance for near clipping
  double distFarClip;

private:
  bool getClippedPointsFovGeneric(const vpPoint &p1, const vpPoint &p2, vpPoint &p1Clipped, vpPoint &p2Clipped,
                                  unsigned int &p1ClippedInfo, unsigned int &p2ClippedInfo, const vpColVector &normal,
                                  const unsigned int &flag);

  bool getClippedPointsDistance(const vpPoint &p1, const vpPoint &p2, vpPoint &p1Clipped, vpPoint &p2Clipped,
                                unsigned int &p1ClippedInfo, unsigned int &p2ClippedInfo, const unsigned int &flag,
                                const double &distance);

public:
  vpPolygon3D();
  vpPolygon3D(const vpPolygon3D &mbtp);
  virtual ~vpPolygon3D();

  void addPoint(const unsigned int n, const vpPoint &P);

  void changeFrame(const vpHomogeneousMatrix &cMo);

  void computePolygonClipped(const vpCameraParameters &cam = vpCameraParameters());

  /*!
    Get the clipping used.

    \sa vpPolygon3DClipping

    \return Clipping flags.
  */
  inline unsigned int getClipping() const { return clippingFlag; }

  /*!
   Get the far distance for clipping.

   \return Far clipping value.
 */
  inline double getFarClippingDistance() const { return distFarClip; }

  /*!
    Return the number of corners.

    \return number of corner of the face
  */
  inline unsigned int getNbPoint() const { return nbpt; }

  /*!
    Return the number of corners at the previous computation.

    \return number of corner of the face at the previous computation
  */
  inline unsigned int getNbCornerInsidePrevImage() const { return nbCornersInsidePrev; }

  unsigned int getNbCornerInsideImage(const vpImage<unsigned char> &I, const vpCameraParameters &cam);

  /*!
    Get the near distance for clipping.

    \return Near clipping value.
  */
  inline double getNearClippingDistance() const { return distNearClip; }

  vpPoint &getPoint(const unsigned int _index);

  std::vector<vpImagePoint> getRoi(const vpCameraParameters &cam);

  std::vector<vpImagePoint> getRoi(const vpCameraParameters &cam, const vpHomogeneousMatrix &cMo);

  void getRoiClipped(const vpCameraParameters &cam, std::vector<vpImagePoint> &roi);

  void getRoiClipped(const vpCameraParameters &cam, std::vector<vpImagePoint> &roi, const vpHomogeneousMatrix &cMo);

  void getRoiClipped(const vpCameraParameters &cam, std::vector<std::pair<vpImagePoint, unsigned int> > &roi);

  void getRoiClipped(const vpCameraParameters &cam, std::vector<std::pair<vpImagePoint, unsigned int> > &roi,
                     const vpHomogeneousMatrix &cMo);

#ifdef VISP_BUILD_DEPRECATED_FUNCTIONS
  /*!
    @name Deprecated functions
  */
  //@{
  vp_deprecated void getRoiClipped(std::vector<vpPoint> &points);
//@}
#endif

  void getPolygonClipped(std::vector<std::pair<vpPoint, unsigned int> > &poly);

  void getPolygonClipped(std::vector<vpPoint> &poly);

  vpPolygon3D &operator=(const vpPolygon3D &mbtp);

  /*!
    Specify which clipping to use.

    \sa vpPolygon3DClipping

    \param flags : New clipping flags.
  */
  inline void setClipping(const unsigned int &flags) { clippingFlag = flags; }

  /*!
    Set the far distance for clipping.

    \param dist : Far clipping value.
  */
  inline void setFarClippingDistance(const double &dist)
  {
    distFarClip = dist;
    clippingFlag = (clippingFlag | vpPolygon3D::FAR_CLIPPING);
  }

  virtual void setNbPoint(const unsigned int nb);

  /*!
    Set the near distance for clipping.

    \param dist : Near clipping value.
  */
  inline void setNearClippingDistance(const double &dist)
  {
    distNearClip = dist;
    clippingFlag = (clippingFlag | vpPolygon3D::NEAR_CLIPPING);
  }

public:
  static void getClippedPolygon(const std::vector<vpPoint> &ptIn, std::vector<vpPoint> &ptOut,
                                const vpHomogeneousMatrix &cMo, const unsigned int &clippingFlags,
                                const vpCameraParameters &cam = vpCameraParameters(), const double &znear = 0.001,
                                const double &zfar = 100);
  static void getMinMaxRoi(const std::vector<vpImagePoint> &roi, int &i_min, int &i_max, int &j_min, int &j_max);
  static bool roiInsideImage(const vpImage<unsigned char> &I, const std::vector<vpImagePoint> &corners);
};

#endif
