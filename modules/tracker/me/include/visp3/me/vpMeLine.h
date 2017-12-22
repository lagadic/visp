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
  \file vpMeLine.h
  \brief Moving edges on a line
*/

#ifndef vpMeLine_HH
#define vpMeLine_HH

#include <visp3/core/vpMath.h>
#include <visp3/core/vpMatrix.h>
#include <visp3/me/vpMeTracker.h>

#include <iostream>
#include <math.h>

/*!
  \class vpMeLine

  \ingroup module_me

  \brief Class that tracks in an image a line moving edges.

  In this class the line is defined by its equation in the \f$ (i,j) =
  (line,column) \f$ image plane. Two kinds of parametrization are available to
  describe a 2D line. The first one corresponds to the following
  equation

  \f[ ai + bj + c = 0 \f]

  where \f$ i \f$ and \f$ j \f$ are the coordinates of the points
  belonging to the line. The line features are \f$ (a, b, c) \f$.

  The second way to write the line equation is to consider polar coordinates
  \f[ i \; cos(\theta) + j \; sin(\theta) - \rho = 0 \f]

  where \f$ i \f$ and \f$ j \f$ are still the coordinates of the
  points belonging to the line. But now the line features are \f$
  (\rho, \theta) \f$. The computation of \f$ \rho \f$ and \f$ \theta
  \f$ is easy thanks to \f$ (a, b, c) \f$.

  \f[ \theta = arctan(b/a) \f]
  \f[ \rho = -c/\sqrt{a^2+b^2} \f]

  The value of \f$ \theta \f$ is between \f$ 0 \f$ and \f$ 2\pi
  \f$. And the value of \f$ \rho \f$ can be positive or negative. The
  conventions to find the right values of the two features are
  illustrated in the following pictures.

  \image html vpMeLine.gif
  \image latex vpMeLine.ps  width=10cm

  The angle \f$\theta\f$ is computed thanks to the direction of the
  arrow. The arrow points to the side of the line which is darker.

  The example below available in tutorial-me-line-tracker.cpp and described
  in \ref tutorial-tracking-me shows how to use this class.

  \include tutorial-me-line-tracker.cpp

  The code below shows how to use this class.
\code
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpImage.h>
#include <visp3/core/vpImagePoint.h>
#include <visp3/me/vpMeLine.h>

int main()
{
  vpImage<unsigned char> I(240, 320);

  // Fill the image with a black rectangle
  I = 0;
  for (int i = 100; i < 180; i ++) {
    for (int j = 120; j < 250; j ++) {
      I[i][j] = 255;
    }
  }

  // Set the moving-edges tracker parameters
  vpMe me;
  me.setRange(25);
  me.setThreshold(15000);
  me.setSampleStep(10);

  // Initialize the moving-edges line tracker parameters
  vpMeLine line;
  line.setMe(&me);

  // Initialize the location of the vertical line to track
  vpImagePoint ip1, ip2; // Two points belonging to the line to track
  ip1.set_i( 120 );
  ip1.set_j( 119 );
  ip2.set_i( 170 );
  ip2.set_j( 122 );

  line.initTracking(I, ip1, ip2);

  while ( 1 )
  {
    // ... Here the code to read or grab the next image.

    // Track the line.
    line.track(I);
  }
  return 0;
}
\endcode

  \note It is possible to display the line as an overlay. For that you
  must use the display function of the class vpMeLine.
*/

class VISP_EXPORT vpMeLine : public vpMeTracker
{
private:
  static void update_indices(double theta, int incr, int i, int j, int &i1, int &i2, int &j1, int &j2);

protected:
  vpMeSite PExt[2];

  double rho, theta;
  double delta, delta_1;
  double angle, angle_1;
  int sign;

  //! Flag to specify wether the intensity of the image at the middle point is
  //! used to compute the sign of rho or not.
  bool _useIntensityForRho;

#ifdef VISP_BUILD_DEPRECATED_FUNCTIONS
public:
#else
protected:
#endif

  double a; //!< Parameter a of the line equation a*i + b*j + c = 0
  double b; //!< Parameter b of the line equation a*i + b*j + c = 0
  double c; //!< Parameter c of the line equation a*i + b*j + c = 0

public:
  vpMeLine();
  vpMeLine(const vpMeLine &meline);
  virtual ~vpMeLine();

  void display(const vpImage<unsigned char> &I, vpColor col);

  void track(const vpImage<unsigned char> &Im);

  void sample(const vpImage<unsigned char> &image);
  void reSample(const vpImage<unsigned char> &I);
  void leastSquare();
  void updateDelta();
  void setExtremities();
  void seekExtremities(const vpImage<unsigned char> &I);
  void suppressPoints();

  void initTracking(const vpImage<unsigned char> &I);
  void initTracking(const vpImage<unsigned char> &I, const vpImagePoint &ip1, const vpImagePoint &ip2);

  void computeRhoTheta(const vpImage<unsigned char> &I);
  double getRho() const;
  double getTheta() const;
  void getExtremities(vpImagePoint &ip1, vpImagePoint &ip2);

  /*!
    Gets the equation parameters of the line
  */
  void getEquationParam(double &A, double &B, double &C)
  {
    A = a;
    B = b;
    C = c;
  }

  /*!
    Gets parameter a of the line equation a*i + b*j + c = 0
  */
  inline double getA() const { return a; }

  /*!
    Gets parameter b of the line equation a*i + b*j + c = 0
  */
  inline double getB() const { return b; }

  /*!
    Gets parameter c of the line equation a*i + b*j + c = 0
  */
  inline double getC() const { return c; }

  static bool intersection(const vpMeLine &line1, const vpMeLine &line2, vpImagePoint &ip);

  /*!
    This method allows to turn off the computation of the sign of the rho
    attribute based on the intensity near the middle point of the line. This
    is usually done to distinguish between a black/white and a white/black
    edge but it may be source of problem (ex. for a servoing example) when
    this point can be occluded.

    \param useIntensityForRho : new value of the flag.
  */
  inline void computeRhoSignFromIntensity(const bool useIntensityForRho) { _useIntensityForRho = useIntensityForRho; }

  // Static Functions
public:
  static void display(const vpImage<unsigned char> &I, const vpMeSite &PExt1, const vpMeSite &PExt2, const double &A,
                      const double &B, const double &C, const vpColor &color = vpColor::green,
                      unsigned int thickness = 1);
  static void display(const vpImage<vpRGBa> &I, const vpMeSite &PExt1, const vpMeSite &PExt2, const double &A,
                      const double &B, const double &C, const vpColor &color = vpColor::green,
                      unsigned int thickness = 1);

  static void display(const vpImage<unsigned char> &I, const vpMeSite &PExt1, const vpMeSite &PExt2,
                      const std::list<vpMeSite> &site_list, const double &A, const double &B, const double &C,
                      const vpColor &color = vpColor::green, unsigned int thickness = 1);
  static void display(const vpImage<vpRGBa> &I, const vpMeSite &PExt1, const vpMeSite &PExt2,
                      const std::list<vpMeSite> &site_list, const double &A, const double &B, const double &C,
                      const vpColor &color = vpColor::green, unsigned int thickness = 1);
};

#endif
