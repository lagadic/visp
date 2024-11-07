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
 * Moving edges.
 */

/*!
 * \file vpMbtMeEllipse.h
 * \brief Moving edges on an ellipse
 */

#ifndef vpMbtMeEllipse_HH
#define vpMbtMeEllipse_HH

#include <list>
#include <math.h>

#include <visp3/core/vpConfig.h>
#include <visp3/me/vpMeEllipse.h>

#ifndef DOXYGEN_SHOULD_SKIP_THIS
BEGIN_VISP_NAMESPACE
/*!
 * \class vpMbtMeEllipse
 * \ingroup group_mbt_features
 *
 * \brief Class that tracks an ellipse moving edges with specific capabilities for
 * model-based tracking.
*/
class VISP_EXPORT vpMbtMeEllipse : public vpMeEllipse
{
public:
  using vpMeTracker::display;

  vpMbtMeEllipse();
  vpMbtMeEllipse(const vpMbtMeEllipse &me_ellipse);

  void computeProjectionError(const vpImage<unsigned char> &_I, double &_sumErrorRad, unsigned int &_nbFeatures,
                              const vpMatrix &SobelX, const vpMatrix &SobelY, bool display, unsigned int length,
                              unsigned int thickness);

  void initTracking(const vpImage<unsigned char> &I, const vpImagePoint &ic, double n20_p, double n11_p, double n02_p,
                    bool doNotTrack, vpImagePoint *pt1 = nullptr, const vpImagePoint *pt2 = nullptr);

  void track(const vpImage<unsigned char> &I);
  void updateParameters(const vpImage<unsigned char> &I, const vpImagePoint &center_p, double n20_p, double n11_p,
                        double n02_p);

private:
  void reSample(const vpImage<unsigned char> &I);
  void sample(const vpImage<unsigned char> &I, bool doNotTrack = false) VP_OVERRIDE;
  void suppressPoints();
};
END_VISP_NAMESPACE
#endif // #ifndef DOXYGEN_SHOULD_SKIP_THIS
#endif
