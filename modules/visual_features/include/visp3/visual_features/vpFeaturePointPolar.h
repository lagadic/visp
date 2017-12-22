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
 * 2D point with polar coordinates visual feature.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/

#ifndef vpFeaturePointPolar_H
#define vpFeaturePointPolar_H

/*!
  \file vpFeaturePointPolar.h
  \brief Class that defines a 2D point visual feature with polar coordinates.
*/

#include <visp3/core/vpMatrix.h>
#include <visp3/core/vpPoint.h>
#include <visp3/visual_features/vpBasicFeature.h>

#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpRGBa.h>

/*!
  \class vpFeaturePointPolar
  \ingroup group_visual_features

  \brief Class that defines 2D image point visual feature with
  polar coordinates \f$(\rho,\theta)\f$ described in \cite Corke09a.

  Let us denote \f$(\rho,\theta)\f$ the polar coordinates of an image
  point, with \f$\rho\f$ the radius of the feature point with respect
  to the optical center and \f$\theta\f$ the angle. From cartesian
  coordinates \f$(x,y)\f$ of a image point, polar coordinates are
  obtained by:

  \f[\rho = \sqrt{x^2+y^2}  \hbox{,}\; \; \theta = \arctan \frac{y}{x}\f]

  From polar coordinates, cartesian coordinates of the feature point
  can be obtained by:

  \f[x = \rho \cos\theta   \hbox{,}\; \; y = \rho \sin\theta\f]

  This class is intended to manipulate the 2D image point visual
  feature in polar coordinates \f$ s = (\rho, \theta) \f$. The
  interaction matrix related to \f$ s \f$ is given by:

  \f[
  L = \left[
  \begin{array}{l}
  L_{\rho} \\
  \; \\
  L_{\theta}\\
  \end{array}
  \right]
  =
  \left[
  \begin{array}{cccccc}
  \frac{-\cos \theta}{Z} & \frac{-\sin \theta}{Z}  &  \frac{\rho}{Z} &
(1+\rho^2)\sin\theta  & -(1+\rho^2)\cos\theta &  0 \\
  \;\\									\
   \frac{\sin\theta}{\rho Z} & \frac{-\cos\theta}{\rho Z} &  0 &  \cos\theta
/\rho &  \sin\theta/\rho & -1 \\ \end{array} \right] \f]

  where \f$Z\f$ is the 3D depth of the considered point in the camera frame.

  Two ways are allowed to initialize the feature.

  - The first way by setting the feature values \f$(\rho,\theta,Z)\f$
    using vpFeaturePointPolar members like set_rho(), set_theta(),
    set_Z(), or set_rhoThetaZ(), or also buildFrom().

  - The second way by using the feature builder functionalities to
    initialize the feature from a dot tracker, like
    vpFeatureBuilder::create (vpFeaturePointPolar &, const
    vpCameraParameters &, const vpDot &) or vpFeatureBuilder::create
    (vpFeaturePointPolar &, const vpCameraParameters &, const vpDot2
    &). Be aware, that in that case only \f$(\rho,\theta)\f$ are
    initialized. You may also initialize the 3D depth \f$Z\f$. It is
    also possible to initialize the feature from a point structure,
    like vpFeatureBuilder::create(vpFeaturePointPolar &, const vpPoint
    &) or vpFeatureBuilder::create(vpFeaturePointPolar &, const
    vpCameraParameters &, const vpCameraParameters &, const vpPoint
    &). In that case all the feature parameters \f$(\rho,\theta,Z)\f$
    would be initialized.

  The interaction() method allows to compute the interaction matrix
  \f$L\f$ associated to the visual feature, while the error() method
  computes the error vector \f$(s - s^*)\f$ between the current visual
  feature and the desired one.

  The code below shows how to create a eye-in hand visual servoing
  task using four 2D point features with polar coordinates. First we
  create four current features \f$s\f$ (p var name in the code) and
  four desired \f$s^*\f$ (pd var name in the code) point features with
  polar coordinates, set the task to use the interaction matrix
  associated to the current feature \f$L_{s}\f$ and than compute the
  camera velocity \f$v=-\lambda \; {L_{s}}^+ \; (s-s^*)\f$. The
  current feature \f$s\f$ is updated in the while() loop, while
  \f$s^*\f$ is initialized at the beginning.

  \code
#include <visp3/core/vpPoint.h>
#include <visp3/visual_features/vpFeatureBuilder.h>
#include <visp3/visual_features/vpFeaturePointPolar.h>
#include <visp3/vs/vpServo.h>

int main()
{

  // Create 4 points to specify the object of interest
  vpPoint point[4];

  // Set the 3D point coordinates in the object frame: oP
  point[0].setWorldCoordinates(-0.1, -0.1, 0);
  point[1].setWorldCoordinates( 0.1, -0.1, 0);
  point[2].setWorldCoordinates( 0.1,  0.1, 0);
  point[3].setWorldCoordinates(-0.1,  0.1, 0);

  // Initialize the desired pose between the camera and the object frame
  vpHomogeneousMatrix cMod;
  cMod.buildFrom(0, 0, 1, 0, 0, 0);

  // Compute the desired position of the point
  for (int i = 0 ; i < 4 ; i++) {
    // Compute the 3D point coordinates in the camera frame cP = cMod * oP
    point[i].changeFrame(cMod);
    // Compute the perspective projection to set (x,y)
    point[i].projection();
  }

  // Create 4 desired visual features as 2D points with polar coordinates
  vpFeaturePointPolar pd[4];
  // Initialize the desired visual feature from the desired point positions
  for (int i = 0 ; i < 4 ; i++)
    vpFeatureBuilder::create(pd[i], point[i]);

  // Initialize the current pose between the camera and the object frame
  vpHomogeneousMatrix cMo;
  cMo.buildFrom(0, 0, 1.2, 0, 0, M_PI);
  // ... cMo need here to be computed from a pose estimation

  for (int i = 0 ; i < 4 ; i++) {
    // Compute the 3D point coordinates in the camera frame cP = cMo * oP
    point[i].changeFrame(cMo);
    // Compute the perspective projection to set (x,y)
    point[i].projection();
  }
  // Create 4 current visual features as 2D points with polar coordinates
  vpFeaturePointPolar p[4];
  // Initialize the current visual feature from the current point positions
  for (int i = 0 ; i < 4 ; i++)
    vpFeatureBuilder::create(p[i], point[i]);

  // Visual servo task initialization
  vpServo task;
  // - Camera is monted on the robot end-effector and velocities are
  //   computed in the camera frame
  task.setServo(vpServo::EYEINHAND_CAMERA);
  // - Interaction matrix is computed with the current visual features s
  task.setInteractionMatrixType(vpServo::CURRENT);
  // - Set the contant gain to 1
  task.setLambda(1);
  // - Add current and desired features
  for (int i = 0 ; i < 4 ; i++)
    task.addFeature(p[i], pd[i]);

  // Control loop
  for ( ; ; ) {
    // ... cMo need here to be estimated from for example a pose estimation.
    // Computes the point coordinates in the camera frame and its 2D
    // coordinates in the image plane
    for (int i = 0 ; i < 4 ; i++)
      point[i].track(cMo) ;

    // Update the current 2D point visual feature with polar coordinates
    for (int i = 0 ; i < 4 ; i++)
      vpFeatureBuilder::create(p[i], point[i]);

    // compute the control law
    vpColVector v = task.computeControlLaw(); // camera velocity
  }

  task.kill();
}
  \endcode

  If you want to deal only with the \f$\rho\f$ subset feature from the 2D
  point feature set, you have just to modify the addFeature() call in the
  previous example by the following line. In that case, the dimension of
  \f$s\f$ is four.

  \code
  // Add the rho subset feature from the 2D point polar coordinates visual features
  task.addFeature(p[i], pd[i], vpFeaturePointPolar::selectRho());
  \endcode

  If you want to build your own control law, this other example shows how
  to create a current (\f$s\f$) and desired (\f$s^*\f$) 2D point visual
  feature with polar coordinates, compute the corresponding error vector
  \f$(s-s^*)\f$ and finally build the interaction matrix \f$L_s\f$.

  \code
#include <visp3/core/vpMatrix.h>
#include <visp3/visual_features/vpFeaturePointPolar.h>

int main()
{
  // Creation of the current feature s
  vpFeaturePointPolar s;
  // Initialize the current feature
  s.buildFrom(0.1, M_PI, 1); // rho=0.1m, theta=pi, Z=1m

  // Creation of the desired feature s
  vpFeaturePointPolar s_star;
  // Initialize the desired feature
  s.buildFrom(0.15, 0, 0.8); // rho=0.15m, theta=0, Z=0.8m

  // Compute the interaction matrix L_s for the current feature
  vpMatrix L = s.interaction();

  // Compute the error vector (s-s*) for the point feature with polar coordinates
  s.error(s_star);

  return 0;
}
  \endcode

*/
class VISP_EXPORT vpFeaturePointPolar : public vpBasicFeature
{
private:
  //! FeaturePoint depth (required to compute the interaction matrix)
  //! default Z = 1m
  double Z;

public:
  // basic constructor
  vpFeaturePointPolar();
  //! Destructor. Does nothing.
  virtual ~vpFeaturePointPolar() {}

  void buildFrom(const double rho, const double theta, const double Z);

  void display(const vpCameraParameters &cam, const vpImage<unsigned char> &I, const vpColor &color = vpColor::green,
               unsigned int thickness = 1) const;
  void display(const vpCameraParameters &cam, const vpImage<vpRGBa> &I, const vpColor &color = vpColor::green,
               unsigned int thickness = 1) const;

  // feature duplication
  vpFeaturePointPolar *duplicate() const;

  // compute the error between two visual features from a subset
  // a the possible features
  vpColVector error(const vpBasicFeature &s_star, const unsigned int select = FEATURE_ALL);

  // basic construction
  void init();

  // get the point rho-coordinates
  double get_rho() const;
  // get the point theta-coordinates
  double get_theta() const;
  // get the point depth (camera frame)
  double get_Z() const;

  // compute the interaction matrix from a subset a the possible features
  vpMatrix interaction(const unsigned int select = FEATURE_ALL);

  // print the name of the feature
  void print(const unsigned int select = FEATURE_ALL) const;

  // set the point rho-coordinates
  void set_rho(const double rho);
  // set the point theta-coordinates
  void set_theta(const double theta);
  // set the point depth (camera frame)
  void set_Z(const double Z);
  // set the point rho, theta polar coordinates and Z coordinate
  void set_rhoThetaZ(const double rho, const double theta, const double Z);

  /*
    vpBasicFeature method instantiation
  */
  static unsigned int selectRho();
  static unsigned int selectTheta();

  /*!
    @name Deprecated functions
  */
  //! compute the error between a visual features and zero
  vpColVector error(const unsigned int select = FEATURE_ALL);
};

#endif

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
