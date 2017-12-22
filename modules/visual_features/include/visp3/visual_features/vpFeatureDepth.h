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
 * Nicolas Melchior
 *
 *****************************************************************************/

#ifndef vpFeatureDepth_H
#define vpFeatureDepth_H

/*!
  \file vpFeatureDepth.h
  \brief Class that defines 3D point visual feature
*/

#include <visp3/core/vpMatrix.h>
#include <visp3/visual_features/vpBasicFeature.h>

#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpRGBa.h>

/*!
  \class vpFeatureDepth
  \ingroup group_visual_features

  \brief Class that defines a 3D point visual feature \f$ s\f$ which
  is composed by one parameters that is \f$ log( \frac{Z}{Z^*}) \f$
  that defines the current depth relative to the desired depth. Here
  \f$ Z \f$ represents the current depth and \f$ Z^* \f$ the desired
  depth.

  In this class \f$ x \f$ and \f$ y \f$ are the 2D coordinates in the
  camera frame and are given in meter. \f$ x \f$, \f$ y \f$ and \f$ Z
  \f$ are needed during the computation of the interaction matrix \f$
  L \f$.

  The visual features can be set easily thanks to the buildFrom() method.

  As the visual feature \f$ s \f$ represents the current depth
  relative to the desired depth, the desired visual feature \f$ s^*
  \f$ is set to zero. Once the value of the visual feature is set, the
  interaction() method allows to compute the interaction matrix \f$ L
  \f$ associated to the visual feature, while the error() method
  computes the error vector \f$(s - s^*)\f$ between the current visual
  feature and the desired one which is here set to zero.

  The code below shows how to create a eye-in hand visual servoing
  task using a 3D depth feature \f$ log( \frac{Z}{Z^*}) \f$ that
  corresponds to the current depth relative to the desired depth. To
  control six degrees of freedom, at least five other features must be
  considered. First we create a current (\f$s\f$) 3D depth
  feature. Then we set the task to use the interaction matrix
  associated to the current feature \f$L_s\f$. And finally we compute
  the camera velocity \f$v=-\lambda \; L_s^+ \; (s-s^*)\f$. The
  current feature \f$s\f$ is updated in the while() loop.

  \code
#include <visp3/visual_features/vpFeatureDepth.h>
#include <visp3/vs/vpServo.h>

int main()
{
  vpServo task; // Visual servoing task

  vpFeatureDepth s; //The current point feature.
  //Set the current parameters x, y, Z and the desired depth Zs
  double x;  //You have to compute the value of x.
  double y;  //You have to compute the value of y.
  double Z;  //You have to compute the value of Z.
  double Zs;  //You have to define the desired depth Zs.
  //Set the point feature thanks to the current parameters.
  s.buildfrom(x, y, Z, log(Z/Zs));

  // Set eye-in-hand control law.
  // The computed velocities will be expressed in the camera frame
  task.setServo(vpServo::EYEINHAND_CAMERA);
  // Interaction matrix is computed with the desired visual features sd
  task.setInteractionMatrixType(vpServo::CURRENT);

  // Add the 3D depth feature to the task
  task.addFeature(s); // s* is here considered as zero

  // Control loop
  for ( ; ; ) {
    // The new parameters x, y and Z must be computed here.

    // Update the current point visual feature
    s.buildfrom(x, y, Z, log(Z/Zs));

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
#include <visp3/core/vpColVector.h>
#include <visp3/core/vpMatrix.h>
#include <visp3/visual_features/vpFeatureDepth.h>

int main()
{
  vpFeatureDepth s; //The current point feature.
  //Set the current parameters x, y, Z and the desired depth Zs
  double x;  //You have to compute the value of x.
  double y;  //You have to compute the value of y.
  double Z;  //You have to compute the value of Z.
  double Zs;  //You have to define the desired depth Zs.
  //Set the point feature thanks to the current parameters.
  s.buildfrom(x, y, Z, log(Z/Zs));

  // Compute the interaction matrix L_s for the current point feature
  vpMatrix L = s.interaction();

  // Compute the error vector (s-s*) for the point feature with s* considered as 0.
  vpColVector s_star(1); // The dimension is 1.
  s_star(1) = 0; // The value of s* is 0.
  s.error(s_star);
}
  \endcode
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
  //! Destructor.
  virtual ~vpFeatureDepth() {}

  /*
    section Set coordinates
  */

  void buildFrom(const double x, const double y, const double Z, const double LogZoverZstar);

  void display(const vpCameraParameters &cam, const vpImage<unsigned char> &I, const vpColor &color = vpColor::green,
               unsigned int thickness = 1) const;
  void display(const vpCameraParameters &cam, const vpImage<vpRGBa> &I, const vpColor &color = vpColor::green,
               unsigned int thickness = 1) const;
  vpFeatureDepth *duplicate() const;
  vpColVector error(const vpBasicFeature &s_star, const unsigned int select = FEATURE_ALL);

  double get_x() const;

  double get_y() const;

  double get_Z() const;

  double get_LogZoverZstar() const;

  void init();
  vpMatrix interaction(const unsigned int select = FEATURE_ALL);

  void print(const unsigned int select = FEATURE_ALL) const;
  void set_x(const double x);

  void set_y(const double y);

  void set_Z(const double Z);

  void set_LogZoverZstar(const double LogZoverZstar);

  void set_xyZLogZoverZstar(const double x, const double y, const double Z, const double logZZs);
};

#endif

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
