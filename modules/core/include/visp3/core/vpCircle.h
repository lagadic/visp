/****************************************************************************
 *
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2019 by Inria. All rights reserved.
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
  \brief Class that defines a 3D circle in the object frame and allows forward projection of a 3D circle in the
  camera frame and in the 2D image plane by perspective projection.
  All the parameters must be set in meter.

  Note that a 3D circle is defined from the intersection between a 3D plane and a 3D sphere.

  A 3D circle has the followings parameters:
  - **in the object frame**: the parameters oA, oB, oC corresponding to the 3D plane with equation
  oA*(x-oX)+oB*(y-oY)+oC*(z-oZ)=0 passing through the 3D sphere center and the
  3D coordinates oX, oY, oZ of the center and radius R of the 3D sphere. These
  parameters registered in vpForwardProjection::oP internal 7-dim vector are set using the constructors
  vpCircle(double oA, double oB, double oC, double oX, double oY, double oZ, double R),
  vpCircle(const vpColVector &oP) or the fonctions
  setWorldCoordinates(double oA, double oB, double oC, double oX, double oY, double oZ, double R)
  and setWorldCoordinates(const vpColVector &oP). To get theses parameters use get_oP().

  - **in the camera frame**: the parameters cA, cB, cC corresponding to the 3D plane cAx+cBy+cCz+D=0
  and the coordinates cX, cY, cZ of the center and radius R of the 3D sphere. These
  parameters registered in vpTracker::cP internal 7-dim vector are computed using
  changeFrame(const vpHomogeneousMatrix &cMo, vpColVector &cP) const or changeFrame(const vpHomogeneousMatrix &cMo).
  These parameters could be retrieved using getA(), getB(), getC(), getX(), getY(), getZ() and getR().
  To get theses parameters use get_cP().

  - **in the image plane**: the center (x, y) and the second order moments mu20, mu11, mu02 of the ellipse corresponding
  to the perspective projection of the 3D circle. These parameters are registered in vpTracker::p internal 5-dim vector
  and computed using projection() and
  projection(const vpColVector &cP, vpColVector &p) const. They could be retrieved using get_x(), get_y(), get_mu20(),
  get_mu11() and get_mu02(). They correspond to 2D normalized circle parameters with values expressed in meters.
  To get theses parameters use get_p().

*/
class VISP_EXPORT vpCircle : public vpForwardProjection
{
public:
  vpCircle();
  explicit vpCircle(const vpColVector &oP);
  vpCircle(double oA, double oB, double oC, double oX, double oY, double oZ, double R);
  virtual ~vpCircle();

  void changeFrame(const vpHomogeneousMatrix &cMo, vpColVector &cP) const;
  void changeFrame(const vpHomogeneousMatrix &cMo);

  void display(const vpImage<unsigned char> &I, const vpCameraParameters &cam, const vpColor &color = vpColor::green,
               unsigned int thickness = 1);
  void display(const vpImage<unsigned char> &I, const vpHomogeneousMatrix &cMo, const vpCameraParameters &cam,
               const vpColor &color = vpColor::green, unsigned int thickness = 1);
  vpCircle *duplicate() const;

  double get_x() const { return p[0]; }
  double get_y() const { return p[1]; }
  double get_mu20() const { return p[2]; }
  double get_mu11() const { return p[3]; }
  double get_mu02() const { return p[4]; }

  double getA() const { return cP[0]; }
  double getB() const { return cP[1]; }
  double getC() const { return cP[2]; }

  double getX() const { return cP[3]; }
  double getY() const { return cP[4]; }
  double getZ() const { return cP[5]; }

  double getR() const { return cP[6]; }

  void projection();
  void projection(const vpColVector &cP, vpColVector &p) const;

  void setWorldCoordinates(const vpColVector &oP);
  void setWorldCoordinates(double oA, double oB, double oC, double oX, double oY, double oZ, double R);

  //###################
  // Static Functions
  //###################

public:
  static void computeIntersectionPoint(const vpCircle &circle, const vpCameraParameters &cam, const double &rho,
                                       const double &theta, double &i, double &j);

protected:
  void init();
};

#endif
