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
 * Sphere feature.
 *
 * Authors:
 * Eric Marchand
 *
 *****************************************************************************/

/*!
  \file vpSphere.h
  \brief  forward projection of a sphere
*/

#ifndef vpSphere_hh
#define vpSphere_hh

#include <visp3/core/vpDebug.h>
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpMath.h>

#include <math.h>
#include <visp3/core/vpForwardProjection.h>
/*!
  \class vpSphere
  \ingroup group_core_geometry
  \brief Class that defines what is a sphere.

  Forward projection of a sphere.
*/
class VISP_EXPORT vpSphere : public vpForwardProjection
{
public:
  void init();
  vpSphere();
  virtual ~vpSphere();

public:
  explicit vpSphere(const vpColVector &oP);
  vpSphere(const double X0, const double Y0, const double Z0, const double R);

  void setWorldCoordinates(const vpColVector &oP);
  void setWorldCoordinates(const double X0, const double Y0, const double Z0, const double R);

  double get_x() const { return p[0]; }
  double get_y() const { return p[1]; }
  double get_mu20() const { return p[2]; }
  double get_mu11() const { return p[3]; }
  double get_mu02() const { return p[4]; }

  double getX() const { return cP[0]; }
  double getY() const { return cP[1]; }
  double getZ() const { return cP[2]; }

  double getR() const { return cP[3]; }

  void projection();
  void projection(const vpColVector &cP, vpColVector &p);
  void changeFrame(const vpHomogeneousMatrix &cMo, vpColVector &cP);
  void changeFrame(const vpHomogeneousMatrix &cMo);

  void display(const vpImage<unsigned char> &I, const vpCameraParameters &cam, const vpColor &color = vpColor::green,
               const unsigned int thickness = 1);
  void display(const vpImage<unsigned char> &I, const vpHomogeneousMatrix &cMo, const vpCameraParameters &cam,
               const vpColor &color = vpColor::green, const unsigned int thickness = 1);

  vpSphere *duplicate() const;
};

#endif
