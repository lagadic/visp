/*
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2023 by Inria. All rights reserved.
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
 * Cylinder feature.
 */

/*!
  \file vpCylinder.h
  \brief  class that defines what is a cylinder
*/

#ifndef vpCylinder_hh
#define vpCylinder_hh

#include <math.h>
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpMath.h>

#include <visp3/core/vpForwardProjection.h>

BEGIN_VISP_NAMESPACE
/*!
 * \class vpCylinder
 * \ingroup group_core_geometry
 * \brief Class that defines a 3D cylinder in the object frame and allows forward projection of a 3D cylinder in the
 * camera frame and in the 2D image plane by perspective projection.
 * All the parameters must be set in meter.
 *
 * A 3D cylinder of radius R is defined by the set of circles of radius R whose center belongs
 * to a straight line perpendicular to the plane of the circles.
 *
 * A 3D cylinder has the followings parameters:
 * - **in the object frame**: the cylinder is represented by the equation:
 *   \f$ (X - oX)^2 + (Y - oY)^2 + (Z - oZ)^2 - (oA \; X + oB \; Y + oC \; Z)^2 -
 *   R^2 = 0 \f$ with
 *   \f[
 *   \left\{ \begin{array}{l}
 *   oA^2 + oB^2 + oC^2 = 1 \\
 *   oA \; oX + oB \; oY + oC \; oZ = 0
 *   \end{array} \right.
 *   \f]
 *   where R is the radius of the cylinder, oA, oB, oC are the
 *   coordinates of its direction vector and oX, oY, oZ are the
 *   coordinates of the nearest point belonging to the cylinder axis from the
 *   projection center.
 *   The corresponding parameters are located in vpForwardProjection::oP 7-dim internal vector. They correspond
 *   to oP = (oA, oB, oC, oX, oY, oZ, R).
 *   Setting the cylinder parameters is achieved through the constructors with
 *   parameters or setWorldCoordinates() methods.
 *   To get theses parameters use get_oP().
 *
 * - **in the camera frame**: parameters are saved in vpTracker::cP 7-dim internal vector
 *   with cP =(cA, cB, cC, cX, cY, cZ, R). Considering the set of parameters oP expressed in the object
 *   frame, cylinder coordinates expressed in the camera frame are obtained using
 *   changeFrame(). To get these parameters use get_cP().
 *
 * - **in the 2D image plane**: parameters are saved in vpTracker::p 4-dim vector.
 *   They correspond to p = (\f$\rho_1\f$, \f$\theta_1\f$, \f$\rho_2\f$, \f$\theta_2\f$), noting
 *   that for non-degenerated cases, the perspective projection of a cylinder on the image plane is a set of two
 *   straight lines with equation:
 *   \f[
 *   \left\{ \begin{array}{lll}
 *   x \;\cos\theta_1 + x \;\sin\theta_1 - \rho_1 = 0 \\
 *   y \;\cos\theta_2 + y \;\sin\theta_2 - \rho_2 = 0
 *   \end{array} \right.
 *   \f]
 *
 * Perspective projection is achieved using projection() methods. The methods
 * get_p(), getRho1(), getTheta1() and getRho2(), getTheta2() allow to access to the
 * projected line parameters.
*/
class VISP_EXPORT vpCylinder : public vpForwardProjection
{
public:
  typedef enum
  {
    line1, /*!< First limb of the cylinder. */
    line2  /*!< Second limb of the cylinder. */
  } vpLineCylinderType;

  vpCylinder();
  VP_EXPLICIT vpCylinder(const vpColVector &oP);
  vpCylinder(double oA, double oB, double oC, double oX, double oY, double oZ, double R);

  void changeFrame(const vpHomogeneousMatrix &cMo, vpColVector &cP) const VP_OVERRIDE;
  void changeFrame(const vpHomogeneousMatrix &cMo) VP_OVERRIDE;

  double computeZ(double x, double y) const;

  void display(const vpImage<unsigned char> &I, const vpCameraParameters &cam, const vpColor &color = vpColor::green,
               unsigned int thickness = 1) VP_OVERRIDE;
  void display(const vpImage<vpRGBa> &I, const vpCameraParameters &cam, const vpColor &color = vpColor::green,
               unsigned int thickness = 1);
  void display(const vpImage<unsigned char> &I, const vpHomogeneousMatrix &cMo, const vpCameraParameters &cam,
               const vpColor &color = vpColor::green, unsigned int thickness = 1) VP_OVERRIDE;
  void display(const vpImage<vpRGBa> &I, const vpHomogeneousMatrix &cMo, const vpCameraParameters &cam,
               const vpColor &color = vpColor::green, unsigned int thickness = 1);

  vpCylinder *duplicate() const VP_OVERRIDE;

/*!
 * Return the \f$\rho_1\f$ parameter of the line corresponding to the
 * projection of the cylinder in the image plane.
 * \sa getTheta1()
 */
  double getRho1() const { return p[0]; }
  /*!
   * Return the \f$\theta_1\f$ parameter of the line corresponding to the
   * projection of the cylinder in the image plane.
   * \sa getRho1()
   */
  double getTheta1() const { return p[1]; }

  /*!
   * Return the \f$\rho_2\f$ parameter of the line corresponding to the
   * projection of the cylinder in the image plane.
   * \sa getTheta2()
   */
  double getRho2() const { return p[2]; }
  /*!
   * Return the \f$\theta_2\f$ parameter of the line corresponding to the
   * projection of the cylinder in the image plane.
   * \sa getRho2()
   */
  double getTheta2() const { return p[3]; }

  /*!
   * Return cylinder cA parameter expressed in the camera frame.
   */
  double getA() const { return cP[0]; }

  /*!
   * Return cylinder cB parameter expressed in the camera frame.
   */
  double getB() const { return cP[1]; }

  /*!
   * Return cylinder cC parameter expressed in the camera frame.
   */
  double getC() const { return cP[2]; }

  /*!
   * Return cylinder cX parameter expressed in the camera frame.
   */
  double getX() const { return cP[3]; }

  /*!
   * Return cylinder cY parameter expressed in the camera frame.
   */
  double getY() const { return cP[4]; }

  /*!
   * Return cylinder cZ parameter expressed in the camera frame.
   */
  double getZ() const { return cP[5]; }

  /*!
   * Return cylinder R parameter corresponding to the cylinder radius.
   */
  double getR() const { return cP[6]; }

  void init() VP_OVERRIDE;

  void projection() VP_OVERRIDE;
  void projection(const vpColVector &cP, vpColVector &p) const VP_OVERRIDE;

  void setWorldCoordinates(const vpColVector &oP) VP_OVERRIDE;
  void setWorldCoordinates(double oA, double oB, double oC, double oX, double oY, double oZ, double R);
};
END_VISP_NAMESPACE
#endif
