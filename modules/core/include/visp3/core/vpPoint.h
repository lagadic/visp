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
 *
 * Description:
 * Point feature.
 */

/*!
  \file vpPoint.h
  \brief  class that defines what is a point
*/

#ifndef VP_POINT_H
#define VP_POINT_H

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpColor.h>
#include <visp3/core/vpForwardProjection.h>
#include <visp3/core/vpMatrix.h>

BEGIN_VISP_NAMESPACE
class vpHomogeneousMatrix;

/*!
  \class vpPoint
  \ingroup group_core_geometry
  \brief Class that defines a 3D point in the object frame and allows forward projection of a 3D point in the
  camera frame and in the 2D image plane by perspective projection.
  All the parameters must be set in meter.

  A 3D point has the followings parameters:
  - **in the object frame**: the normalized 3D coordinates oX, oY, oZ, oW of the point. These
    parameters registered in vpForwardProjection::oP internal 4-dim vector are set using the constructors vpPoint(double
    oX, double oY, double oZ), vpPoint(const vpColVector &oP) and vpPoint(const std::vector<double> &oP) or the functions
    setWorldCoordinates(double oX, double oY, double oZ),
    setWorldCoordinates(const vpColVector &oP) and setWorldCoordinates(const std::vector<double> &oP).
    To get theses parameters use get_oP().

  - **in the camera frame**: the normalized coordinates cX, cY, cZ, 1 of the point. These
    parameters registered in vpTracker::cP internal 4-dim vector are computed using
    changeFrame(const vpHomogeneousMatrix &cMo, vpColVector &cP) const or changeFrame(const vpHomogeneousMatrix &cMo).
    These parameters could be retrieved using get_X(), get_Y() and get_Z().
    To get theses parameters use get_cP().

  - **in the image plane**: the 2D normalized coordinates (x, y, 1) corresponding
    to the perspective projection of the point. These parameters are registered in vpTracker::p internal 3-dim vector and
    computed using projection() and projection(const vpColVector &cP, vpColVector &p) const. They could be retrieved using
    get_x() and get_y(). They correspond to 2D normalized point parameters with values expressed in meters. To get theses
    parameters use get_p().

*/
class VISP_EXPORT vpPoint : public vpForwardProjection
{

public:
  //! Basic constructor.
  vpPoint();
  vpPoint(double oX, double oY, double oZ);
  VP_EXPLICIT vpPoint(const vpColVector &oP);
  VP_EXPLICIT vpPoint(const std::vector<double> &oP);

public:
  // Compute the 3D coordinates _cP  (camera frame)
  void changeFrame(const vpHomogeneousMatrix &cMo, vpColVector &cP) const VP_OVERRIDE;
  void changeFrame(const vpHomogeneousMatrix &cMo) VP_OVERRIDE;

  void display(const vpImage<unsigned char> &I, const vpCameraParameters &cam, const vpColor &color = vpColor::green,
               unsigned int thickness = 1) VP_OVERRIDE;
  void display(const vpImage<unsigned char> &I, const vpHomogeneousMatrix &cMo, const vpCameraParameters &cam,
               const vpColor &color = vpColor::green, unsigned int thickness = 1) VP_OVERRIDE;

  void display(const vpImage<vpRGBa> &I, const vpCameraParameters &cam, const vpColor &color = vpColor::green,
               unsigned int thickness = 1);
  void display(const vpImage<vpRGBa> &I, const vpHomogeneousMatrix &cMo, const vpCameraParameters &cam,
               const vpColor &color = vpColor::green, unsigned int thickness = 1);

  vpPoint *duplicate() const VP_OVERRIDE;

  // Get coordinates
  double get_X() const;
  double get_Y() const;
  double get_Z() const;
  double get_W() const;
  double get_oX() const;
  double get_oY() const;
  double get_oZ() const;
  double get_oW() const;
  double get_x() const;
  double get_y() const;
  double get_w() const;

  void getWorldCoordinates(double &oX, double &oY, double &oZ);
  void getWorldCoordinates(vpColVector &oP);
  vpColVector getWorldCoordinates(void);
  void getWorldCoordinates(std::vector<double> &oP);

  friend VISP_EXPORT std::ostream &operator<<(std::ostream &os, const vpPoint &vpp);

  //! Projection onto the image plane of a point. Input: the 3D coordinates in
  //! the camera frame _cP, output : the 2D coordinates _p.

  void projection(const vpColVector &_cP, vpColVector &_p) const VP_OVERRIDE;
  void projection() VP_OVERRIDE;

  // Set coordinates
  void set_X(double cX);
  void set_Y(double cY);
  void set_Z(double cZ);
  void set_W(double cW);
  void set_oX(double oX);
  void set_oY(double oY);
  void set_oZ(double oZ);
  void set_oW(double oW);
  void set_x(double x);
  void set_y(double y);
  void set_w(double w);

  void setWorldCoordinates(double oX, double oY, double oZ);

  void setWorldCoordinates(const vpColVector &oP) VP_OVERRIDE;
  void setWorldCoordinates(const std::vector<double> &oP);

protected:
  //! Basic construction.
  void init() VP_OVERRIDE;
};
END_VISP_NAMESPACE
#endif
