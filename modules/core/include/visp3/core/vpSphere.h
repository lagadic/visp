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
  \brief Class that defines a 3D sphere in the object frame and allows forward projection of a 3D sphere in the
  camera frame and in the 2D image plane by perspective projection.
  All the parameters must be set in meter.

  A sphere has the followings parameters:
  - **in the object frame**: the 3D coordinates oX, oY, oZ of the center and radius R. These
  parameters registered in vpForwardProjection::oP internal 4-dim vector are set using the constructors vpSphere(double oX, double oY, double oZ, double R),
  vpSphere(const vpColVector &oP) or the fonctions setWorldCoordinates(double oX, double oY, double oZ, double R)
  and setWorldCoordinates(const vpColVector &oP).
  To get theses parameters use get_oP().

  - **in the camera frame**: the coordinates cX, cY, cZ of the center and radius R. These
  parameters registered in vpTracker::cP internal 4-dim vector are computed using
  changeFrame(const vpHomogeneousMatrix &cMo, vpColVector &cP) const or changeFrame(const vpHomogeneousMatrix &cMo).
  These parameters could be retrieved using getX(), getY(), getZ() and getR().
  To get theses parameters use get_cP().

  - **in the image plane**: here we consider the parameters of the ellipse corresponding
  to the perspective projection of the 3D sphere. The parameters are the ellipse centroid (x, y)
  and n20, n11, n02 which are the second order centered moments of
  the ellipse normalized by its area (i.e., such that \f$n_{ij} = \mu_{ij}/a\f$ where
  \f$\mu_{ij}\f$ are the centered moments and a the area).
  These parameters are registered in vpTracker::p internal 5-dim vector and computed using projection() and
  projection(const vpColVector &cP, vpColVector &p) const. They could be retrieved using get_x(), get_y(), get_n20(),
  get_n11() and get_n02(). They correspond to 2D normalized sphere parameters with values expressed in meters.
  To get theses parameters use get_p().
*/
class VISP_EXPORT vpSphere : public vpForwardProjection
{
public:
  vpSphere();
  explicit vpSphere(const vpColVector &oP);
  vpSphere(double oX, double oY, double oZ, double R);
  virtual ~vpSphere();

  void changeFrame(const vpHomogeneousMatrix &cMo, vpColVector &cP) const;
  void changeFrame(const vpHomogeneousMatrix &cMo);

  void display(const vpImage<unsigned char> &I, const vpCameraParameters &cam, const vpColor &color = vpColor::green,
               unsigned int thickness = 1);
  void display(const vpImage<unsigned char> &I, const vpHomogeneousMatrix &cMo, const vpCameraParameters &cam,
               const vpColor &color = vpColor::green, unsigned int thickness = 1);

  vpSphere *duplicate() const;

  double get_x() const { return p[0]; }
  double get_y() const { return p[1]; }

  double get_n20() const { return p[2]; }
  double get_n11() const { return p[3]; }
  double get_n02() const { return p[4]; }

  double getX() const { return cP[0]; }
  double getY() const { return cP[1]; }
  double getZ() const { return cP[2]; }
  double getR() const { return cP[3]; }

  void projection();
  void projection(const vpColVector &cP, vpColVector &p) const;

  void setWorldCoordinates(const vpColVector &oP);
  void setWorldCoordinates(double oX, double oY, double oZ, double R);

protected:
  void init();

public:
#if defined(VISP_BUILD_DEPRECATED_FUNCTIONS)
  /*!
    @name Deprecated functions
  */
  //@{
  /*!
   * \deprecated You should rather use get_n20().
   * This function is incorrectly named and is confusing since it
   * returns second order centered moments of the ellipse normalized
   * by its area that corresponds to \f$n_20 = mu_20/a\f$.
   */
  vp_deprecated double get_mu20() const { return p[2]; }
  /*!
   * \deprecated You should rather use get_n11().
   * This function is incorrectly named and is confusing since it
   * returns second order centered moments of the ellipse normalized
   * by its area that corresponds to \f$n_11 = mu_11/a\f$.
   */
  vp_deprecated double get_mu11() const { return p[3]; }
  /*!
   * \deprecated You should rather use get_n02().
   * This function is incorrectly named and is confusing since it
   * returns second order centered moments of the ellipse normalized
   * by its area that corresponds to \f$n_02 = mu_02/a\f$.
   */
  vp_deprecated double get_mu02() const { return p[4]; }
  //@}
#endif
};

#endif
