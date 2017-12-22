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
 * 2D point visual feature.
 *
 * Authors:
 * Eric Marchand
 *
 *****************************************************************************/

#ifndef vpFeaturePoint_H
#define vpFeaturePoint_H

/*!
  \file vpFeaturePoint.h
  \brief Class that defines 2D point visual feature
*/

#include <visp3/core/vpMatrix.h>
#include <visp3/core/vpPoint.h>
#include <visp3/visual_features/vpBasicFeature.h>

#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpRGBa.h>

/*!
  \class vpFeaturePoint
  \ingroup group_visual_features

  \brief Class that defines a 2D point visual feature \f$ s\f$ which
  is composed by two parameters that are the cartesian coordinates \f$
  x \f$ and \f$ y \f$.

  In this class \f$ x \f$ and \f$ y \f$ are the 2D coordinates in the
  image plane and are given in meter. \f$ Z \f$ which is the 3D
  coordinate representing the depth is also a parameter of the
  point. It is needed during the computation of the interaction matrix
  \f$ L \f$.

  The visual features can be set easily from an instance of the
  classes vpPoint, vpDot or vpDot2. For more precision see the
  vpFeatureBuilder class.

  Once the values of the visual features are set, the interaction()
  method allows to compute the interaction matrix \f$ L \f$ associated
  to the visual feature, while the error() method computes the error
  vector \f$(s - s^*)\f$ between the current visual feature and the
  desired one.

  The code below shows how to create a eye-in hand visual servoing
  task using a 2D point feature \f$(x,y)\f$ that correspond to the 2D
  coordinates of a point in the image plane. To control six degrees
  of freedom, at least four other features must be considered like two
  other point features for example. First we create a current
  (\f$s\f$) 2D point feature. Then we set the task to use the
  interaction matrix associated to the current feature \f$L_s\f$. And
  finally we compute the camera velocity \f$v=-\lambda \; L_s^+ \;
  (s-s^*)\f$. The current feature \f$s\f$ is updated in the while()
  loop.

  \code
#include <visp3/visual_features/vpFeaturePoint.h>
#include <visp3/vs/vpServo.h>

int main()
{
  vpServo task; // Visual servoing task

  vpFeaturePoint sd; //The desired point feature.
  //Set the desired features x and y
  double xd = 0;
  double yd = 0;
  //Set the depth of the point in the camera frame.
  double Zd = 1;
  //Set the point feature thanks to the desired parameters.
  sd.buildFrom(xd, yd, Zd);

  vpFeaturePoint s; //The current point feature.
  //Set the current features x and y
  double x;  //You have to compute the value of x.
  double y;  //You have to compute the value of y.
  double Z;  //You have to compute the value of Z.
  //Set the point feature thanks to the current parameters.
  s.buildFrom(x, y, Z);
  //In this case the parameter Z is not necessary because the interaction matrix is computed
  //with the desired visual feature.

  // Set eye-in-hand control law.
  // The computed velocities will be expressed in the camera frame
  task.setServo(vpServo::EYEINHAND_CAMERA);
  // Interaction matrix is computed with the desired visual features sd
  task.setInteractionMatrixType(vpServo::DESIRED);

  // Add the 2D point feature to the task
  task.addFeature(s, sd);

  // Control loop
  for ( ; ; ) {
    // The new parameters x and y must be computed here.

    // Update the current point visual feature
    s.buildFrom(x, y, Z);

    // compute the control law
    vpColVector v = task.computeControlLaw(); // camera velocity
  }
  return 0;
}
  \endcode

  If you want to build your own control law, this other example shows how
  to create a current (\f$s\f$) and desired (\f$s^*\f$) 2D point visual
  feature, compute the corresponding error vector \f$(s-s^*)\f$ and finally
  build the interaction matrix \f$L_s\f$.

  \code
#include <visp3/core/vpMatrix.h>
#include <visp3/visual_features/vpFeaturePoint.h>

int main()
{
  vpFeaturePoint sd; //The desired point feature.
  //Set the desired features x and y
  double xd = 0;
  double yd = 0;
  //Set the depth of the point in the camera frame.
  double Zd = 1;
  //Set the point feature thanks to the desired parameters.
  sd.buildFrom(xd, yd, Zd);

  vpFeaturePoint s; //The current point feature.
  //Set the current features x and y
  double x;  //You have to compute the value of x.
  double y;  //You have to compute the value of y.
  double Z;  //You have to compute the value of Z.
  //Set the point feature thanks to the current parameters.
  s.buildFrom(x, y, Z);

  // Compute the interaction matrix L_s for the current point feature
  vpMatrix L = s.interaction();
  // You can also compute the interaction matrix L_s for the desired point feature
  // The corresponding line of code is : vpMatrix L = sd.interaction();

  // Compute the error vector (s-sd) for the point feature
  s.error(s_star);
}
  \endcode

  An other fully explained example is given in the \ref tutorial-ibvs.

*/

class VISP_EXPORT vpFeaturePoint : public vpBasicFeature
{
private:
  //! FeaturePoint depth (required to compute the interaction matrix)
  //! default Z = 1m
  double Z;

public:
  vpFeaturePoint();
  //! Destructor.
  virtual ~vpFeaturePoint() {}

  void buildFrom(const double x, const double y, const double Z);

  void display(const vpCameraParameters &cam, const vpImage<unsigned char> &I, const vpColor &color = vpColor::green,
               unsigned int thickness = 1) const;
  void display(const vpCameraParameters &cam, const vpImage<vpRGBa> &I, const vpColor &color = vpColor::green,
               unsigned int thickness = 1) const;

  vpFeaturePoint *duplicate() const;

  vpColVector error(const vpBasicFeature &s_star, const unsigned int select = FEATURE_ALL);
  //! Compute the error between a visual features and zero
  vpColVector error(const unsigned int select = FEATURE_ALL);

  double get_x() const;

  double get_y() const;

  double get_Z() const;

  void init();
  vpMatrix interaction(const unsigned int select = FEATURE_ALL);

  void print(const unsigned int select = FEATURE_ALL) const;

  void set_x(const double x);

  void set_y(const double y);

  void set_Z(const double Z);

  void set_xyZ(const double x, const double y, const double Z);

  // feature selection
  static unsigned int selectX();
  static unsigned int selectY();

  /*!
    @name Deprecated functions
  */
  typedef enum {
    X = 1, // x coordinates
    Y = 2  // y coordinates
  } vpFeaturePointType;
};

#endif
