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
 * Point feature.
 *
 * Authors:
 * Eric Marchand
 *
 *****************************************************************************/

#ifndef vpPoint_H
#define vpPoint_H

/*!
  \file vpPoint.h
  \brief  class that defines what is a point
*/

class vpHomogeneousMatrix;

#include <visp3/core/vpColor.h>
#include <visp3/core/vpForwardProjection.h>
#include <visp3/core/vpMatrix.h>

/*!
  \class vpPoint
  \ingroup group_core_geometry
  \brief Class that defines what is a point.
*/
class VISP_EXPORT vpPoint : public vpForwardProjection
{

public:
  //! Basic constructor.
  vpPoint();
  vpPoint(double oX, double oY, double oZ);
  explicit vpPoint(const vpColVector &P);
  explicit vpPoint(const std::vector<double> &P);
  //! Destructor.
  virtual ~vpPoint() {}

public:
  // Compute the 3D coordinates _cP  (camera frame)
  void changeFrame(const vpHomogeneousMatrix &cMo, vpColVector &_cP);
  void changeFrame(const vpHomogeneousMatrix &cMo);

  void display(const vpImage<unsigned char> &I, const vpCameraParameters &cam, const vpColor &color = vpColor::green,
               const unsigned int thickness = 1);
  void display(const vpImage<unsigned char> &I, const vpHomogeneousMatrix &cMo, const vpCameraParameters &cam,
               const vpColor &color = vpColor::green, const unsigned int thickness = 1);
  void display(const vpImage<vpRGBa> &I, const vpHomogeneousMatrix &cMo, const vpCameraParameters &cam,
               const vpColor &color = vpColor::green, const unsigned int thickness = 1);
  vpPoint *duplicate() const;

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
  void getWorldCoordinates(vpColVector &P);
  vpColVector getWorldCoordinates(void);
  void getWorldCoordinates(std::vector<double> &P);

  //! Basic construction.
  void init();

  friend VISP_EXPORT std::ostream &operator<<(std::ostream &os, const vpPoint &vpp);
  vpPoint &operator=(const vpPoint &vpp);

  //! Projection onto the image plane of a point. Input: the 3D coordinates in
  //! the camera frame _cP, output : the 2D coordinates _p.
  void projection(const vpColVector &_cP, vpColVector &_p);

  void projection();

  // Set coordinates
  void set_X(const double X);
  void set_Y(const double Y);
  void set_Z(const double Z);
  void set_W(const double W);
  void set_oX(const double oX);
  void set_oY(const double oY);
  void set_oZ(const double oZ);
  void set_oW(const double oW);
  void set_x(const double x);
  void set_y(const double y);
  void set_w(const double w);

  void setWorldCoordinates(const double oX, const double oY, const double oZ);
  void setWorldCoordinates(const vpColVector &P);
  void setWorldCoordinates(const std::vector<double> &P);
};

#endif
