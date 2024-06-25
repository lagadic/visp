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
 * \file vpFeaturePoint.h
 * \brief Class that defines 2D point visual feature
 */

#ifndef vpFeaturePoint_H
#define vpFeaturePoint_H

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpMatrix.h>
#include <visp3/core/vpPoint.h>
#include <visp3/visual_features/vpBasicFeature.h>

#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpRGBa.h>

BEGIN_VISP_NAMESPACE
/*!
 * \class vpFeaturePoint
 * \ingroup group_visual_features
 *
 * \brief Class that defines a 2D point visual feature \f$ s\f$ which
 * is composed by two parameters that are the cartesian coordinates \f$
 * x \f$ and \f$ y \f$.
 *
 * In this class \f$ x \f$ and \f$ y \f$ are the 2D coordinates in the
 * image plane and are given in meter. \f$ Z \f$ which is the 3D
 * coordinate representing the depth is also a parameter of the
 * point. It is needed during the computation of the interaction matrix
 * \f$ L \f$.
 *
 * The visual features can be set easily from an instance of the
 * classes vpPoint, vpDot or vpDot2. For more precision see the
 * vpFeatureBuilder class.
 *
 * Once the values of the visual features are set, the interaction()
 * method allows to compute the interaction matrix \f$ L \f$ associated
 * to the visual feature, while the error() method computes the error
 * vector \f$(s - s^*)\f$ between the current visual feature and the
 * desired one.
 *
 * The code below shows how to create a eye-in hand visual servoing
 * task using a 2D point feature \f$(x,y)\f$ that correspond to the 2D
 * coordinates of a point in the image plane. To control six degrees
 * of freedom, at least four other features must be considered like two
 * other point features for example. First we create a current
 * (\f$s\f$) 2D point feature. Then we set the task to use the
 * interaction matrix associated to the current feature \f$L_s\f$. And
 * finally we compute the camera velocity \f$v=-\lambda \; L_s^+ \;
 * (s-s^*)\f$. The current feature \f$s\f$ is updated in the while()
 * loop.
 *
 * \code
 * #include <visp3/visual_features/vpFeaturePoint.h>
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
 *   vpFeaturePoint sd; //The desired point feature.
 *   // Set the desired features x and y
 *   double xd = 0;
 *   double yd = 0;
 *   // Set the depth of the point in the camera frame.
 *   double Zd = 1;
 *   // Set the point feature thanks to the desired parameters.
 *   sd.build(xd, yd, Zd);
 *
 *   vpFeaturePoint s; //The current point feature.
 *   // Set the current features x and y
 *   double x;  // You have to compute the value of x.
 *   double y;  // You have to compute the value of y.
 *   double Z;  // You have to compute the value of Z.
 *   // Set the point feature thanks to the current parameters.
 *   s.build(x, y, Z);
 *   // In this case the parameter Z is not necessary because the interaction matrix is computed
 *   // with the desired visual feature.
 *
 *   // Set eye-in-hand control law.
 *   // The computed velocities will be expressed in the camera frame
 *   task.setServo(vpServo::EYEINHAND_CAMERA);
 *   // Interaction matrix is computed with the desired visual features sd
 *   task.setInteractionMatrixType(vpServo::DESIRED);
 *
 *   // Add the 2D point feature to the task
 *   task.addFeature(s, sd);
 *
 *   // Control loop
 *   for ( ; ; ) {
 *     // The new parameters x and y must be computed here.
 *
 *     // Update the current point visual feature
 *     s.build(x, y, Z);
 *
 *     // Compute the control law
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
 * #include <visp3/core/vpMatrix.h>
 * #include <visp3/visual_features/vpFeaturePoint.h>
 *
 * #ifdef ENABLE_VISP_NAMESPACE
 * using namespace VISP_NAMESPACE_NAME;
 * #endif
 *
 * int main()
 * {
 *   vpFeaturePoint sd; //The desired point feature.
 *   // Set the desired features x and y
 *   double xd = 0;
 *   double yd = 0;
 *   // Set the depth of the point in the camera frame.
 *   double Zd = 1;
 *   // Set the point feature thanks to the desired parameters.
 *   sd.build(xd, yd, Zd);
 *
 *   vpFeaturePoint s; //The current point feature.
 *   // Set the current features x and y
 *   double x;  // You have to compute the value of x.
 *   double y;  // You have to compute the value of y.
 *   double Z;  // You have to compute the value of Z.
 *   // Set the point feature thanks to the current parameters.
 *   s.build(x, y, Z);
 *
 *   // Compute the interaction matrix L_s for the current point feature
 *   vpMatrix L = s.interaction();
 *   // You can also compute the interaction matrix L_s for the desired point feature
 *   // The corresponding line of code is : vpMatrix L = sd.interaction();
 *
 *   // Compute the error vector (s-sd) for the point feature
 *   s.error(s_star);
 * }
 * \endcode
 *
 * An other fully explained example is given in the \ref tutorial-ibvs.
*/
class VISP_EXPORT vpFeaturePoint : public vpBasicFeature
{
private:
  //! FeaturePoint depth (required to compute the interaction matrix)
  //! default Z = 1m
  double Z;

public:
  vpFeaturePoint();

#ifdef VISP_BUILD_DEPRECATED_FUNCTIONS
  void buildFrom(double x, double y, double Z);
#endif
  vpFeaturePoint &build(const double &x, const double &y, const double &Z);

  void display(const vpCameraParameters &cam, const vpImage<unsigned char> &I, const vpColor &color = vpColor::green,
               unsigned int thickness = 1) const VP_OVERRIDE;
  void display(const vpCameraParameters &cam, const vpImage<vpRGBa> &I, const vpColor &color = vpColor::green,
               unsigned int thickness = 1) const VP_OVERRIDE;

  vpFeaturePoint *duplicate() const VP_OVERRIDE;

  vpColVector error(const vpBasicFeature &s_star, unsigned int select = FEATURE_ALL) VP_OVERRIDE;

  double get_x() const;

  double get_y() const;

  double get_Z() const;

  void init() VP_OVERRIDE;
  vpMatrix interaction(unsigned int select = FEATURE_ALL) VP_OVERRIDE;
  void print(unsigned int select = FEATURE_ALL) const VP_OVERRIDE;

  void set_x(double x);

  void set_y(double y);

  void set_Z(double Z);

  void set_xyZ(double x, double y, double Z);

  // feature selection
  static unsigned int selectX();
  static unsigned int selectY();

  /*!
    @name Deprecated functions
  */
  typedef enum
  {
    X = 1, // x coordinates
    Y = 2  // y coordinates
  } vpFeaturePointType;
};
END_VISP_NAMESPACE
#endif
