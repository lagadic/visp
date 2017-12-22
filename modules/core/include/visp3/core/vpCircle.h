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
 * Visual feature circle.
 *
 * Authors:
 * Eric Marchand
 *
 *****************************************************************************/

/*!
  \file vpCircle.h
  \brief  class that defines what is a circle
*/

#ifndef vpCircle_hh
#define vpCircle_hh

#include <math.h>
#include <visp3/core/vpDebug.h>
#include <visp3/core/vpForwardProjection.h>
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpMath.h>

/*!
  \class vpCircle
  \ingroup group_core_geometry
  \brief Class that defines what is a circle.
*/
class VISP_EXPORT vpCircle : public vpForwardProjection
{
public:
  void init();
  vpCircle();
  explicit vpCircle(const vpColVector &oP);
  vpCircle(const double A, const double B, const double C, const double X0, const double Y0, const double Z0,
           const double R);
  virtual ~vpCircle();

  void setWorldCoordinates(const vpColVector &oP);
  void setWorldCoordinates(const double A, const double B, const double C, const double X0, const double Y0,
                           const double Z0, const double R);

  double getA() const { return cP[0]; }
  double getB() const { return cP[1]; }
  double getC() const { return cP[2]; }

  double getX() const { return cP[3]; }
  double getY() const { return cP[4]; }
  double getZ() const { return cP[5]; }

  double getR() const { return cP[6]; }

  void projection();
  void projection(const vpColVector &cP, vpColVector &p);
  void changeFrame(const vpHomogeneousMatrix &cMo, vpColVector &cP);
  void changeFrame(const vpHomogeneousMatrix &cMo);

  void display(const vpImage<unsigned char> &I, const vpCameraParameters &cam, const vpColor &color = vpColor::green,
               const unsigned int thickness = 1);
  void display(const vpImage<unsigned char> &I, const vpHomogeneousMatrix &cMo, const vpCameraParameters &cam,
               const vpColor &color = vpColor::green, const unsigned int thickness = 1);
  vpCircle *duplicate() const;

  //###################
  // Static Functions
  //###################

public:
  static void computeIntersectionPoint(const vpCircle &circle, const vpCameraParameters &cam, const double &rho,
                                       const double &theta, double &i, double &j);
};

#endif
