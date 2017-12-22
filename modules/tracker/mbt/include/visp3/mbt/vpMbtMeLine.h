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
 * Implementation of a line used by the model-based tracker.
 *
 * Authors:
 * Nicolas Melchior
 * Romain Tallonneau
 * Eric Marchand
 *
 *****************************************************************************/

/*!
 \file vpMbtMeLine.h
 \brief Implementation of a line used by the model-based tracker.
*/

#ifndef vpMbtMeLine_HH
#define vpMbtMeLine_HH

#include <visp3/core/vpPoint.h>
#include <visp3/me/vpMe.h>
#include <visp3/me/vpMeTracker.h>

#ifndef DOXYGEN_SHOULD_SKIP_THIS

/*!
  \class vpMbtMeLine
  \brief Implementation of a line used by the model-based tracker.
  \ingroup group_mbt_features

 */
class VISP_EXPORT vpMbtMeLine : public vpMeTracker
{
private:
  vpMeSite PExt[2];
  double rho, theta, theta_1;
  double delta, delta_1;
  int sign;
  double a, b, c;

public:
  int imin, imax;
  int jmin, jmax;
  double expecteddensity;

public:
  vpMbtMeLine();
  ~vpMbtMeLine();

  void computeProjectionError(const vpImage<unsigned char> &_I, double &_sumErrorRad, unsigned int &_nbFeatures);

  void display(const vpImage<unsigned char> & /*I*/, vpColor /*col*/) { ; }
  void display(const vpImage<unsigned char> &I)
  {
    vpMeTracker::display(I);
  } // Shouldn't be here since it's already in vpMeTracker

  /*!
  Get the a coefficient of the line corresponding to \f$ i \; cos(\theta) + j
  \; sin(\theta) - \rho = 0 \f$

  \return : The a coefficient of the moving edge
 */
  inline double get_a() const { return this->a; }

  /*!
  Get the a coefficient of the line corresponding to \f$ i \; cos(\theta) + j
  \; sin(\theta) - \rho = 0 \f$

  \return : The b coefficient of the moving edge
 */
  inline double get_b() const { return this->b; }

  /*!
  Get the a coefficient of the line corresponding to \f$ i \; cos(\theta) + j
  \; sin(\theta) - \rho = 0 \f$

  \return : The c coefficient of the moving edge
 */
  inline double get_c() const { return this->c; }

  void initTracking(const vpImage<unsigned char> &I, const vpImagePoint &ip1, const vpImagePoint &ip2, double rho,
                    double theta);

  void track(const vpImage<unsigned char> &I);

  void updateParameters(const vpImage<unsigned char> &I, double rho, double theta);
  void updateParameters(const vpImage<unsigned char> &I, const vpImagePoint &ip1, const vpImagePoint &ip2, double rho,
                        double theta);

private:
  void bubbleSortI();
  void bubbleSortJ();
  void sample(const vpImage<unsigned char> &image);
  void seekExtremities(const vpImage<unsigned char> &I);
  void setExtremities();
  void suppressPoints(const vpImage<unsigned char> &I);
  void reSample(const vpImage<unsigned char> &image);
  void reSample(const vpImage<unsigned char> &image, const vpImagePoint &ip1, const vpImagePoint &ip2);
  void updateDelta();
};

#endif // #ifndef DOXYGEN_SHOULD_SKIP_THIS

#endif
