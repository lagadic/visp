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
 * Implementation of a line used by the model-based tracker.
 */

/*!
 * \file vpMbtMeLine.h
 * \brief Implementation of a line used by the model-based tracker.
 */

#ifndef VP_MBT_ME_LINE_H
#define VP_MBT_ME_LINE_H

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpPoint.h>
#include <visp3/me/vpMe.h>
#include <visp3/me/vpMeLine.h>

#ifndef DOXYGEN_SHOULD_SKIP_THIS

BEGIN_VISP_NAMESPACE
/*!
 * \class vpMbtMeLine
 * \brief Implementation of a line used by the model-based tracker.
 * \ingroup group_mbt_features
*/
class VISP_EXPORT vpMbtMeLine : public vpMeLine
{
public:
  int imin, imax;
  int jmin, jmax;
  double expecteddensity;

public:
  vpMbtMeLine();
  vpMbtMeLine(const vpMbtMeLine &meline);

  void computeProjectionError(const vpImage<unsigned char> &_I, double &_sumErrorRad, unsigned int &_nbFeatures,
                              const vpMatrix &SobelX, const vpMatrix &SobelY, bool display, unsigned int length,
                              unsigned int thickness);

  void display(const vpImage<unsigned char> & /* I */, const vpColor &/* col */, unsigned int /* thickness */) { ; }
  using vpMeTracker::display;

  /*!
   * Get the a coefficient of the line corresponding to \f$ i \; cos(\theta) + j
   * \; sin(\theta) - \rho = 0 \f$
   *
   * \return : The a coefficient of the moving edge
   */
  inline double get_a() const { return m_a; }

  /*!
   * Get the a coefficient of the line corresponding to \f$ i \; cos(\theta) + j
   * \; sin(\theta) - \rho = 0 \f$
   *
   * \return : The b coefficient of the moving edge
   */
  inline double get_b() const { return m_b; }

  /*!
   * Get the a coefficient of the line corresponding to \f$ i \; cos(\theta) + j
   * \; sin(\theta) - \rho = 0 \f$
   *
   * \return : The c coefficient of the moving edge
   */
  inline double get_c() const { return m_c; }

  void initTracking(const vpImage<unsigned char> &I, const vpImagePoint &ip1, const vpImagePoint &ip2, double rho,
                    double theta, bool doNoTrack);

  void track(const vpImage<unsigned char> &I);

  void updateParameters(const vpImage<unsigned char> &I, double rho, double theta);
  void updateParameters(const vpImage<unsigned char> &I, const vpImagePoint &ip1, const vpImagePoint &ip2, double rho,
                        double theta);

private:
  virtual unsigned int seekExtremities(const vpImage<unsigned char> &I) VP_OVERRIDE;

  void suppressPoints(const vpImage<unsigned char> &I);
  // void reSample(const vpImage<unsigned char> &image);
  void reSample(const vpImage<unsigned char> &image, const vpImagePoint &ip1, const vpImagePoint &ip2);
};
END_VISP_NAMESPACE
#endif // #ifndef DOXYGEN_SHOULD_SKIP_THIS

#endif
