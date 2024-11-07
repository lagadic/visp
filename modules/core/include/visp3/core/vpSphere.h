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
 * Sphere feature.
 */

/*!
 * \file vpSphere.h
 * \brief forward projection of a sphere
 */

#ifndef VP_SPHERE_H
#define VP_SPHERE_H

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpMath.h>

#include <math.h>
#include <visp3/core/vpForwardProjection.h>

BEGIN_VISP_NAMESPACE
/*!
 * \class vpSphere
 * \ingroup group_core_geometry
 * \brief Class that defines a 3D sphere in the object frame and allows forward projection of a 3D sphere in the
 * camera frame and in the 2D image plane by perspective projection.
 * All the parameters must be set in meter.
 *
 * A sphere has the followings parameters:
 * - **in the object frame**: the 3D coordinates oX, oY, oZ of the center and radius R. These
 *   parameters registered in vpForwardProjection::oP internal 4-dim vector are set using the constructors vpSphere(double
 *   oX, double oY, double oZ, double R), vpSphere(const vpColVector &oP) or the functions setWorldCoordinates(double oX,
 *   double oY, double oZ, double R) and setWorldCoordinates(const vpColVector &oP). To get theses parameters use get_oP().
 *
 * - **in the camera frame**: the coordinates cX, cY, cZ of the center and radius R. These
 *   parameters registered in vpTracker::cP internal 4-dim vector are computed using
 *   changeFrame(const vpHomogeneousMatrix &cMo, vpColVector &cP) const or changeFrame(const vpHomogeneousMatrix &cMo).
 *   These parameters could be retrieved using getX(), getY(), getZ() and getR().
 *   To get theses parameters use get_cP().
 *
 * - **in the image plane**: here we consider the parameters of the ellipse corresponding
 *   to the perspective projection of the 3D sphere. The parameters are the ellipse centroid (x, y)
 *   and n20, n11, n02 which are the second order centered moments of
 *   the ellipse normalized by its area (i.e., such that \f$n_{ij} = \mu_{ij}/a\f$ where
 *   \f$\mu_{ij}\f$ are the centered moments and a the area).
 *   These parameters are registered in vpTracker::p internal 5-dim vector and computed using projection() and
 *   projection(const vpColVector &cP, vpColVector &p) const. They could be retrieved using get_x(), get_y(), get_n20(),
 *   get_n11() and get_n02(). They correspond to 2D normalized sphere parameters with values expressed in meters.
 *   To get theses parameters use get_p().
*/
class VISP_EXPORT vpSphere : public vpForwardProjection
{
public:
  vpSphere();
  VP_EXPLICIT vpSphere(const vpColVector &oP);
  vpSphere(double oX, double oY, double oZ, double R);

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

  vpSphere *duplicate() const VP_OVERRIDE;

  double get_x() const { return p[0]; }
  double get_y() const { return p[1]; }

  double get_n20() const { const unsigned int index_2 = 2; return p[index_2]; }
  double get_n11() const { const unsigned int index_3 = 3; return p[index_3]; }
  double get_n02() const { const unsigned int index_4 = 4; return p[index_4]; }

  double getX() const { const unsigned int index_0 = 0; return cP[index_0]; }
  double getY() const { const unsigned int index_1 = 1; return cP[index_1]; }
  double getZ() const { const unsigned int index_2 = 2; return cP[index_2]; }
  double getR() const { const unsigned int index_3 = 3; return cP[index_3]; }


  void projection() VP_OVERRIDE;
  void projection(const vpColVector &cP, vpColVector &p) const VP_OVERRIDE;
  void setWorldCoordinates(const vpColVector &oP) VP_OVERRIDE;

  void setWorldCoordinates(double oX, double oY, double oZ, double R);

public:
#if defined(VISP_BUILD_DEPRECATED_FUNCTIONS)
  /*!
   * @name Deprecated functions
   */
   //@{
   /*!
    * \deprecated You should rather use get_n20().
    * This function is incorrectly named and is confusing since it
    * returns second order centered moments of the ellipse normalized
    * by its area that corresponds to \f$n_20 = mu_20/a\f$.
    */
  VP_DEPRECATED double get_mu20() const { const unsigned int index_2 = 2; return p[index_2]; }
  /*!
   * \deprecated You should rather use get_n11().
   * This function is incorrectly named and is confusing since it
   * returns second order centered moments of the ellipse normalized
   * by its area that corresponds to \f$n_11 = mu_11/a\f$.
   */
  VP_DEPRECATED double get_mu11() const { const unsigned int index_3 = 3; return p[index_3]; }
  /*!
   * \deprecated You should rather use get_n02().
   * This function is incorrectly named and is confusing since it
   * returns second order centered moments of the ellipse normalized
   * by its area that corresponds to \f$n_02 = mu_02/a\f$.
   */
  VP_DEPRECATED double get_mu02() const { const unsigned int index_4 = 4; return p[index_4]; }
  //@}
#endif
protected:
  void init() VP_OVERRIDE;

};
END_VISP_NAMESPACE
#endif
