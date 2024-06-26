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
 * Line feature.
 */

/*!
 * \file vpLine.h
 * \brief  class that defines what is a line
 */

#ifndef VP_LINE_H
#define VP_LINE_H

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpMatrix.h>

#include <visp3/core/vpForwardProjection.h>

BEGIN_VISP_NAMESPACE
/*!
 * \class vpLine
 * \ingroup group_core_geometry
 *
 * \brief Class that defines a 3D line in the object frame and allows forward projection
 * of the line in the camera frame and in the 2D image plane by perspective projection.
 * All the parameters must be set in meter.
 *
 * Note that a 3D line is defined from the intersection between two 3D planes.
 *
 * A 3D line has the followings parameters:
 * - **in the 3D object frame**: parameters are located in vpForwardProjection::oP 8-dim internal vector. They correspond
 *   to the parameters oA1, oB1, oC1, oD1 and oA2, oB2, oC2, oD2 defining the equations of the two planes.
 *   Each point \f$ (X, Y, Z) \f$ which belongs to the 3D line is a solution of those two
 *   equations:
 *   \f[ oA1*X + oB1*Y + oC1*Z + oD1 = 0 \f]
 *   \f[ oA2*X + oB2*Y + oC2*Z + oD2 = 0 \f]
 *   To update these line parameters you may use setWorldCoordinates(). To get theses parameters use get_oP().
 *
 * - **in the 3D camera frame**: parameters are saved in vpTracker::cP 8-dim internal vector. They correspond
 *   to the parameters cA1, cB1, cC1, cD1 and cA2, cB2, cC2, cD2 defining the equations of the two planes.
 *   Each point \f$ (X, Y, Z) \f$ which belongs to the 3D line is a solution of those two
 *   equations:
 *   \f[ cA1*X + cB1*Y + cC1*Z + cD1 = 0 \f]
 *   \f[ cA2*X + cB2*Y + cC2*Z + cD2 = 0 \f]
 *   It is easily possible to compute these parameters thanks to the corresponding 3D parameters oP in the
 *   object frame. But you have to note that four constraints are added in the planes equations.
 *   \f[ cD1 = 0 \f]
 *   \f[ cD2 > 0 \f]
 *   \f[ cA1*cA2 + cB1*cB2 + cC1*cC2 = 0 \f]
 *   \f[ || cA2 || = 1 \f]
 *   To compute these parameters you may use changeFrame(). To get these parameters use get_cP().
 *
 * - **in the 2D image plane**: parameters are saved in vpTracker::p 2-dim vector. They correspond
 *   to the parameters (\f$\rho\f$, \f$\theta\f$). These
 *   2D parameters are obtained from the perspective projection of the 3D line parameters expressed
 *   in the camera frame. They are defined thanks to the 2D equation of a line.
 *   \f[ x \; cos(\theta) + y \; sin(\theta) -\rho = 0 \f] Here \f$ x
 *   \f$ and \f$ y \f$ are the coordinates of a point belonging to the
 *   line in the image plane while \f$ \rho \f$ and \f$ \theta \f$ are
 *   the parameters used to define the line. The value of \f$ \theta
 *   \f$ is between \f$ -\pi/2 \f$ and \f$ \pi/2 \f$ and the value of
 *   \f$ \rho \f$ can be positive or negative. The conventions used to
 *   choose the sign of \f$ \rho \f$ and the value of \f$ \theta \f$
 *   are illustrated by the following image.
 *   \image html vpFeatureLine.gif
 *   \image latex vpFeatureLine.ps width=10cm
 *
 * The line parameters corresponding to the image frame are located
 * in the vpTracker::p public attribute, where \e p is a vector defined
 * as: \f[ p = \left[\begin{array}{c} \rho \\ \theta \end{array}\right] \f]
 * To compute these parameters use projection(). To get the corresponding values use get_p().
*/
class VISP_EXPORT vpLine : public vpForwardProjection
{
public:
  vpLine();

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

  vpLine *duplicate() const VP_OVERRIDE;


  /*!
   * Gets the \f$ \rho \f$ value corresponding to one of the
   * two parameters used to define the line parametrization in the
   * image plane.
   *
   * \return Returns the current value of \f$ \rho \f$.
   *
   * \sa getTheta()
   */
  double getRho() const { return p[0]; }

  /*!
   * Gets the \f$ \theta \f$ angle value corresponding to one of the
   * two parameters used to define the line parametrization in the
   * image plane.
   *
   * \return Returns the current value of \f$ \theta \f$.
   *
   * \sa getRho()
   */
  double getTheta() const { return p[1]; }

  /*!
   * Sets the \f$ \rho \f$ parameter used to define the line in the
   * image plane.
   *
   * \param rho : The desired value for \f$ \rho \f$.
   *
   * \sa setTheta()
   */
  void setRho(double rho) { p[0] = rho; }

  /*!
   * Sets the \f$ \theta \f$ angle value used to define the line in the
   * image plane.
   *
   * \param theta : The desired value for \f$ \theta \f$ angle.
   *
   * \sa setRho()
   */
  void setTheta(double theta) { p[1] = theta; }

  void setWorldCoordinates(const double &oA1, const double &oB1, const double &oC1, const double &oD1,
                           const double &oA2, const double &oB2, const double &oC2, const double &oD2);

  void setWorldCoordinates(const vpColVector &oP1, const vpColVector &oP2);

  void setWorldCoordinates(const vpColVector &oP) VP_OVERRIDE;
  void projection() VP_OVERRIDE;
  void projection(const vpColVector &cP, vpColVector &p) const VP_OVERRIDE;

protected:
  void init() VP_OVERRIDE;
};
END_VISP_NAMESPACE
#endif
