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
 * Sphere feature.
 *
 * Authors:
 * Eric Marchand
 *
 *****************************************************************************/

#include <visp3/core/vpFeatureDisplay.h>
#include <visp3/core/vpSphere.h>

void vpSphere::init()
{

  oP.resize(4);
  cP.resize(4);

  p.resize(5);
}

void vpSphere::setWorldCoordinates(const vpColVector &oP_) { this->oP = oP_; }

void vpSphere::setWorldCoordinates(const double X0, const double Y0, const double Z0, const double R)
{
  oP[0] = X0;
  oP[1] = Y0;
  oP[2] = Z0;
  oP[3] = R;
}

vpSphere::vpSphere() { init(); }

vpSphere::vpSphere(const vpColVector &oP_)
{
  init();
  setWorldCoordinates(oP_);
}

vpSphere::vpSphere(const double X0, const double Y0, const double Z0, const double R)
{
  init();
  setWorldCoordinates(X0, Y0, Z0, R);
}

vpSphere::~vpSphere() {}

//! perspective projection of the sphere
void vpSphere::projection() { projection(cP, p); }

//! Perspective projection of the circle.
void vpSphere::projection(const vpColVector &cP_, vpColVector &p_)
{
  double x0, y0, z0; // variables intermediaires
  //   double k0, k1, k2, k3, k4;  //variables intermediaires
  double E, A, B; // variables intermediaires

  // calcul des parametres M20, M11, M02 de l'ellipse
  double s, r; // variables intermediaires
  r = cP_[3];

  x0 = cP_[0];
  y0 = cP_[1];
  z0 = cP_[2];

  s = r * r - y0 * y0 - z0 * z0;

  //   k0 = (r*r - x0*x0 -z0*z0)/s;
  //   k1 = (x0*y0)/s;
  //   k2 = (x0*z0)/s;
  //   k3 = (y0*z0)/s;
  //   k4 = (r*r - x0*x0 -y0*y0)/s;

  if ((s = z0 * z0 - r * r) < 0.0) {
    vpERROR_TRACE("sphere derriere le plan image\n");
  }

  p_[0] = x0 * z0 / s; // x
  p_[1] = y0 * z0 / s; // y

  if (fabs(x0) > 1e-6) {
    //   vpERROR_TRACE(" %f",r) ;
    double e = y0 / x0;
    double b = r / sqrt(s);
    double a = x0 * x0 + y0 * y0 + z0 * z0 - r * r;
    if (a < 0.0) {
      vpERROR_TRACE("sphere derriere le plan image\n");
    }
    a = r * sqrt(a) / s;
    if (fabs(e) <= 1.0) {
      E = e;
      A = a;
      B = b;
    } else {
      E = -1.0 / e;
      A = b;
      B = a;
    }
  } else {
    //   vpERROR_TRACE(" %f",r) ;
    E = 0.0;
    A = r / sqrt(s);
    B = r * sqrt(y0 * y0 + z0 * z0 - r * r) / s;
  }

  p_[2] = (A * A + B * B * E * E) / (1.0 + E * E); // mu20
  p_[3] = (A * A - B * B) * E / (1.0 + E * E);     // mu11
  p_[4] = (B * B + A * A * E * E) / (1.0 + E * E); // mu02

  // vpERROR_TRACE(" %f",r) ;

  //  std::cout << p.t() ;
}
//! perspective projection of the circle
void vpSphere::changeFrame(const vpHomogeneousMatrix &cMo) { changeFrame(cMo, cP); }

//! Perspective projection of the circle.
void vpSphere::changeFrame(const vpHomogeneousMatrix &cMo, vpColVector &cP_)
{
  double x0, y0, z0; // variables intermediaires

  x0 = cMo[0][0] * oP[0] + cMo[0][1] * oP[1] + cMo[0][2] * oP[2] + cMo[0][3];
  y0 = cMo[1][0] * oP[0] + cMo[1][1] * oP[1] + cMo[1][2] * oP[2] + cMo[1][3];
  z0 = cMo[2][0] * oP[0] + cMo[2][1] * oP[1] + cMo[2][2] * oP[2] + cMo[2][3];

  cP_[3] = oP[3];

  cP_[0] = x0;
  cP_[1] = y0;
  cP_[2] = z0;
}

//! for memory issue (used by the vpServo class only)
vpSphere *vpSphere::duplicate() const
{
  vpSphere *feature = new vpSphere(*this);
  return feature;
}

// non destructive wrt. cP and p
void vpSphere::display(const vpImage<unsigned char> &I, const vpHomogeneousMatrix &cMo, const vpCameraParameters &cam,
                       const vpColor &color, const unsigned int thickness)
{
  vpColVector _cP, _p;
  changeFrame(cMo, _cP);
  projection(_cP, _p);
  vpFeatureDisplay::displayEllipse(_p[0], _p[1], _p[2], _p[3], _p[4], cam, I, color, thickness);
}

void vpSphere::display(const vpImage<unsigned char> &I, const vpCameraParameters &cam, const vpColor &color,
                       const unsigned int thickness)
{
  vpFeatureDisplay::displayEllipse(p[0], p[1], p[2], p[3], p[4], cam, I, color, thickness);
}
