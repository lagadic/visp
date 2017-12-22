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
 * Moving edges.
 *
 * Authors:
 * Eric Marchand
 *
 *****************************************************************************/

/*!
  \file vpMbtMeEllipse.h
  \brief Moving edges on an ellipse
*/

#ifndef vpMbtMeEllipse_HH
#define vpMbtMeEllipse_HH

#include <visp3/core/vpColVector.h>
#include <visp3/core/vpMatrix.h>

#include <visp3/core/vpImagePoint.h>
#include <visp3/me/vpMeSite.h>
#include <visp3/me/vpMeTracker.h>

#include <visp3/core/vpColor.h>
#include <visp3/core/vpImage.h>

#include <list>
#include <math.h>

#ifndef DOXYGEN_SHOULD_SKIP_THIS

/*!
  \class vpMbtMeEllipse
  \ingroup group_mbt_features

  \brief Class that tracks an ellipse moving edges.

  In this class, an ellipse is defined as the set of points \f$ (i,j) \f$ of
  the image frame (For more information about the image frame see the
  vpImagePoint documentation) that satisfy the implicit equation :

  \f[ i^2 + K_0j^2 + 2K_1ij + 2K_2i + 2K_3j + K4 = 0 \f]

  If \f$ K_0 \f$ is equal to 1 and \f$ K_1 \f$ is equal to 0 the the set of
  points \f$ (i,j) \f$ represents a circle.

  The five parameters are stored in the public attribute K.

  An ellipse is also defined thanks to three other parameter which are \f$ a
  \f$, \f$ b \f$ and \f$ e \f$. \f$ a \f$ represents the semiminor axis and
  \f$ b \f$ is the semimajor axis. Here \f$ e \f$ is the angle made by the
  major axis and the i axis of the image frame \f$ (i,j) \f$. The following
  figure shows better meaning of those parameters.

  \image html vpMeEllipse.gif
  \image latex vpMeEllipse.ps  width=10cm

  It is possible to compute the coordinates \f$ (i,j) \f$ of a point which
  belongs to the ellipse thanks to the following equations :

  \f[ i = i_c + b cos(e) cos(\alpha) - a sin(e) sin(\alpha) \f]
  \f[ j = j_c + b sin(e) cos(\alpha) + a cos(e) sin(\alpha) \f]

  Here the coordinates \f$ (i_c,j_c) \f$ are the coordinates of the ellipse
  center in the image frame and \f$ \alpha \f$ is an angle beetween \f$
  [0,2\pi] \f$ and which enables to describe all the points of the ellipse.

  \image html vpMeEllipse2.gif
  \image latex vpMeEllipse2.ps  width=10cm

  The example below available in tutorial-me-ellipse-tracker.cpp and described
  in \ref tutorial-tracking-me, section \ref tracking_me_ellipse shows how to
  use this class.

  \include tutorial-me-ellipse-tracker.cpp

  The code below shows how to use this class.
\code
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpImage.h>
#include <visp3/core/vpImagePoint.h>
#include <visp3/mbt/vpMbtMeEllipse.h>

int main()
{
  vpImage<unsigned char> I;

  // I is the image containing the ellipse to track

  // Set the moving-edges tracker parameters
  vpMe me;
  me.setRange(25);
  me.setPointsToTrack(20);
  me.setThreshold(15000);
  me.setSampleStep(10);

  // Initialize the moving-edges ellipse tracker parameters
  vpMbtMeEllipse ellipse;
  ellipse.setMe(&me);

  // Initialize the tracking. You have to click on five different points belonging to the ellipse.
  ellipse.initTracking(I);

  while ( 1 )
  {
    // ... Here the code to read or grab the next image.

    // Track the ellipse.
    ellipse.track(I);
  }
  return 0;
}
\endcode

  \note It is possible to display the ellipse as an overlay. For that you
  must use the display function of the class vpMbtMeEllipse.
*/

class VISP_EXPORT vpMbtMeEllipse : public vpMeTracker
{
public:
  vpMbtMeEllipse();
  vpMbtMeEllipse(const vpMbtMeEllipse &meellipse);
  virtual ~vpMbtMeEllipse();

  void computeProjectionError(const vpImage<unsigned char> &_I, double &_sumErrorRad, unsigned int &_nbFeatures);

  void display(const vpImage<unsigned char> &I, vpColor col);
  void display(const vpImage<unsigned char> &I)
  {
    vpMeTracker::display(I);
  } // Shouldn't be here since it's already in vpMeTracker
  /*!
    \return Expected number of moving edges to track along the ellipse.
   */
  int getExpectedDensity() { return (int)expecteddensity; }

  /*!
    Gets the 2 order central moment \f$ \mu_{11} \f$.

    \return the value of \f$ \mu_{11} \f$.
  */
  inline double get_mu11() const { return mu11; }

  /*!
    Gets the 2 order central moment \f$ \mu_{02} \f$.

    \return the value of \f$ \mu_{02} \f$.
  */
  inline double get_mu02() const { return mu02; }

  /*!
    Gets the 2 order central moment \f$ \mu_{20} \f$.

    \return the value of \f$ \mu_{20} \f$.
  */
  inline double get_mu20() const { return mu20; }

  /*!
    Gets the center of the ellipse.
  */
  inline vpImagePoint getCenter() const { return iPc; }

  /*!
    Gets the semiminor axis of the ellipse.
  */
  inline double getA() const { return a; }

  /*!
    Gets the semimajor axis of the ellipse.
  */
  inline double getB() const { return b; }

  /*!
    Gets the angle made by the major axis and the i axis of the image frame
    \f$ (i,j) \f$
  */
  inline double getE() const { return e; }

  /*!
    Gets the parameters a, b, e of the ellipse.
  */
  void getEquationParam(double &A, double &B, double &E)
  {
    A = a;
    B = b;
    E = e;
  }

  void initTracking(const vpImage<unsigned char> &I, const vpImagePoint &ic, double mu20_p, double mu11_p,
                    double mu02_p);

  void track(const vpImage<unsigned char> &Im);

  void updateParameters(const vpImage<unsigned char> &I, const vpImagePoint &ic, double mu20_p, double mu11_p,
                        double mu02_p);

protected:
  //! The coordinates of the ellipse center.
  vpImagePoint iPc;
  //! \f$ a \f$ is the semiminor axis of the ellipse.
  double a;
  //! \f$ b \f$ is the semimajor axis of the ellipse.
  double b;
  //! \f$ e \f$ is the angle made by the major axis and the i axis of the
  //! image frame \f$ (i,j) \f$.
  double e;

protected:
  //! Value of cos(e).
  double ce;
  //! Value of sin(e).
  double se;

  //! Second order central moments
  double mu11, mu20, mu02;
  //! Threshold for the robust least square.
  double thresholdWeight;
  //! Expected number of me to track along the ellipse.
  double expecteddensity;

private:
  void sample(const vpImage<unsigned char> &image);
  void reSample(const vpImage<unsigned char> &I);
  void updateTheta();
  void suppressPoints();
};

#endif // #ifndef DOXYGEN_SHOULD_SKIP_THIS
#endif
