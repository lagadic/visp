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
 * 2D point visual feature.
 */

/*!
 * \file vpFeatureDepth.h
 * \brief Class that defines 3D point visual feature
 */

#ifndef vpFeatureDepth_H
#define vpFeatureDepth_H

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpMatrix.h>
#include <visp3/visual_features/vpBasicFeature.h>

#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpRGBa.h>

BEGIN_VISP_NAMESPACE
/*!
 * \class vpFeatureDepth
 * \ingroup group_visual_features
 *
 * \brief Class that defines a 3D point visual feature \f$ s\f$ which
 * is composed by one parameters that is \f$ log( \frac{Z}{Z^*}) \f$
 * that defines the current depth relative to the desired depth. Here
 * \f$ Z \f$ represents the current depth and \f$ Z^* \f$ the desired
 * depth.
 *
 * In this class \f$ x \f$ and \f$ y \f$ are the 2D coordinates in the
 * camera frame and are given in meter. \f$ x \f$, \f$ y \f$ and \f$ Z
 * \f$ are needed during the computation of the interaction matrix \f$
 * L \f$.
 *
 * The visual features can be set easily thanks to the build() method.
 *
 * As the visual feature \f$ s \f$ represents the current depth
 * relative to the desired depth, the desired visual feature \f$ s^*
 * \f$ is set to zero. Once the value of the visual feature is set, the
 * interaction() method allows to compute the interaction matrix \f$ L
 * \f$ associated to the visual feature, while the error() method
 * computes the error vector \f$(s - s^*)\f$ between the current visual
 * feature and the desired one which is here set to zero.
 *
 * The code below shows how to create a eye-in hand visual servoing
 * task using a 3D depth feature \f$ log( \frac{Z}{Z^*}) \f$ that
 * corresponds to the current depth relative to the desired depth. To
 * control six degrees of freedom, at least five other features must be
 * considered. First we create a current (\f$s\f$) 3D depth
 * feature. Then we set the task to use the interaction matrix
 * associated to the current feature \f$L_s\f$. And finally we compute
 * the camera velocity \f$v=-\lambda \; L_s^+ \; (s-s^*)\f$. The
 * current feature \f$s\f$ is updated in the while() loop.
 *
 * \code
 * #include <visp3/visual_features/vpFeatureDepth.h>
 * #include <visp3/vs/vpServo.h>
 *
 * #ifdef ENABLE_VISP_NAMESPACE
 * using namespace VISP_NAMESPACE_NAME;
 * #endif
 *
 * int main()
 * {
 *   vpServo task; // Visual servoing task
 *
 *   vpFeatureDepth s; //The current point feature.
 *   //Set the current parameters x, y, Z and the desired depth Zs
 *   double x;   // You have to compute the value of x.
 *   double y;   // You have to compute the value of y.
 *   double Z;   // You have to compute the value of Z.
 *   double Zs;  // You have to define the desired depth Zs.
 *   //Set the point feature thanks to the current parameters.
 *   s.build(x, y, Z, log(Z/Zs));
 *
 *   // Set eye-in-hand control law.
 *   // The computed velocities will be expressed in the camera frame
 *   task.setServo(vpServo::EYEINHAND_CAMERA);
 *   // Interaction matrix is computed with the desired visual features sd
 *   task.setInteractionMatrixType(vpServo::CURRENT);
 *
 *   // Add the 3D depth feature to the task
 *   task.addFeature(s); // s* is here considered as zero
 *
 *   // Control loop
 *   for ( ; ; ) {
 *     // The new parameters x, y and Z must be computed here.
 *
 *     // Update the current point visual feature
 *     s.build(x, y, Z, log(Z/Zs));
 *
 *     // compute the control law
 *     vpColVector v = task.computeControlLaw(); // camera velocity
 *   }
 *   return 0;
 * }
 * \endcode
 *
 * If you want to build your own control law, this other example shows how
 * to create a current (\f$s\f$) and desired (\f$s^*\f$) 2D point visual
 * feature, compute the corresponding error vector \f$(s-s^*)\f$ and finally
 * build the interaction matrix \f$L_s\f$.
 *
 * \code
 * #include <visp3/core/vpColVector.h>
 * #include <visp3/core/vpMatrix.h>
 * #include <visp3/visual_features/vpFeatureDepth.h>
 *
 * #ifdef ENABLE_VISP_NAMESPACE
 * using namespace VISP_NAMESPACE_NAME;
 * #endif
 *
 * int main()
 * {
 *   vpFeatureDepth s; //The current point feature.
 *   //Set the current parameters x, y, Z and the desired depth Zs
 *   double x;   // You have to compute the value of x.
 *   double y;   // You have to compute the value of y.
 *   double Z;   // You have to compute the value of Z.
 *   double Zs;  // You have to define the desired depth Zs.
 *   //Set the point feature thanks to the current parameters.
 *   s.build(x, y, Z, log(Z/Zs));
 *
 *   // Compute the interaction matrix L_s for the current point feature
 *   vpMatrix L = s.interaction();
 *
 *   // Compute the error vector (s-s*) for the point feature with s* considered as 0.
 *   vpColVector s_star(1); // The dimension is 1.
 *   s_star(1) = 0; // The value of s* is 0.
 *   s.error(s_star);
 * }
 * \endcode
*/
class VISP_EXPORT vpFeatureDepth : public vpBasicFeature
{

private:
  //! The \f$ x \f$ 2D coordinate of the point in the camera frame (required
  //! to compute the interaction matrix)
  double x;
  //! The \f$ y \f$ 2D coordinate of the point in the camera frame (required
  //! to compute the interaction matrix)
  double y;
  //! The \f$ Z \f$ 3D coordinate of the point in the camera frame (required
  //! to compute the interaction matrix)
  double Z;

public:
  vpFeatureDepth();

  /*
    section Set coordinates
  */

#ifdef VISP_BUILD_DEPRECATED_FUNCTIONS
  void buildFrom(double x, double y, double Z, double LogZoverZstar);
#endif
  vpFeatureDepth &build(const double &x, const double &y, const double &Z, const double &LogZoverZstar);

  void display(const vpCameraParameters &cam, const vpImage<unsigned char> &I, const vpColor &color = vpColor::green,
               unsigned int thickness = 1) const VP_OVERRIDE;
  void display(const vpCameraParameters &cam, const vpImage<vpRGBa> &I, const vpColor &color = vpColor::green,
               unsigned int thickness = 1) const VP_OVERRIDE;
  vpFeatureDepth *duplicate() const VP_OVERRIDE;
  vpColVector error(const vpBasicFeature &s_star, unsigned int select = FEATURE_ALL) VP_OVERRIDE;

  double get_x() const;

  double get_y() const;

  double get_Z() const;

  double get_LogZoverZstar() const;

  void init() VP_OVERRIDE;
  vpMatrix interaction(unsigned int select = FEATURE_ALL) VP_OVERRIDE;
  void print(unsigned int select = FEATURE_ALL) const VP_OVERRIDE;

  void set_x(double x);

  void set_y(double y);

  void set_Z(double Z);

  void set_LogZoverZstar(double LogZoverZstar);

  void set_xyZLogZoverZstar(double x, double y, double Z, double logZZs);
};
END_VISP_NAMESPACE
#endif
